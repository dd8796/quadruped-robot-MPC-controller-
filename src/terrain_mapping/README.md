# terrain_mapping — robot-centric elevation map

Builds and continuously publishes a **local, robot-centric elevation map** from the RGBD camera
point cloud (D435; simulated in Gazebo). Each cell holds a height and a variance, fused by a
**per-cell Kalman filter**. The published map also carries a post-processed layer (hole filling +
median), in which [`quadru_control`](../quadru_control/README.md) chooses the footholds.

This node does **one thing**: maintain the map. It does not detect steps and makes no decisions;
the controller queries the map on every leg swing. A staircase is not a special mode, just
terrain.

Principle: ETH RSL / ANYbotics *elevation_mapping* (Fankhauser et al. 2018, "Probabilistic Terrain
Mapping for Mobile Robots with Uncertain Localization") and the *grid_map* library (Fankhauser &
Hutter 2016), rebuilt without their dependencies (kindr).

---

## Contents

1. [ROS 2 data flow](#1-ros-2-data-flow)
2. [Interfaces: topics, TF, parameters](#2-interfaces-topics-tf-parameters)
3. [Point cloud pipeline](#3-point-cloud-pipeline)
4. [Equations](#4-equations)
5. [Published layers and their consumers](#5-published-layers-and-their-consumers)
6. [Files and functions](#6-files-and-functions)
7. [Tests](#7-tests)
8. [Run and visualize](#8-run-and-visualize)
9. [Design choices and limitations](#9-design-choices-and-limitations)

---

## 1. ROS 2 data flow

```
 Gazebo (depth camera, 30 Hz)
   └─► /chien/camera/points  (PointCloud2, camera_link_optical frame)
                                   │
 joint_state_broadcaster ─► /joint_states ─► robot_state_publisher ─► TF trunk → legN_*, camera_link_optical
 CheetahController ─► /mevius_mpc_controller/odom ─► odom_to_tf.py ─► TF odom → trunk
                                   │
                                   ▼
          ┌──────────────── terrain_mapping_node ────────────────┐
          │ 1. recenter the map on the robot (TF odom→trunk)      │
          │ 2. prediction: variance += q·Δt (all cells)           │
          │ 3. range filter (camera frame)                        │
          │ 4. transform → odom                                   │
          │ 5. finite, in-map points, VoxelGrid                   │
          │ 6. robot self-filter (legs, trunk)                    │
          │ 7. plausibility filter (trunk height)                 │
          │ 8. per-cell Kalman fusion                             │
          │ 9. hole filling + median → elevation_filtered, publish│
          └───────────────────────┬───────────────────────────────┘
                                  ├─► /terrain_mapping_node/elevation_map   (GridMap)
                                  │        └─► CheetahController (footholds, ground height)
                                  └─► /terrain_mapping_node/elevation_cloud (PointCloud2) ─► RViz
```

All processing happens in the cloud callback (single thread, `rclcpp::spin`): one map is
published per received cloud (~30 Hz in Gazebo).

## 2. Interfaces: topics, TF, parameters

### 2.1 Subscribed topic

| Topic | Type | QoS | Content |
|---|---|---|---|
| `/chien/camera/points` | `sensor_msgs/PointCloud2` | SensorData | depth camera cloud, `camera_link_optical` frame (z = depth axis), 640 × 480, 30 Hz, simulated range [0.1, 10] m |

### 2.2 Published topics

| Topic | Type | Frame | Content |
|---|---|---|---|
| `~/elevation_map` = `/terrain_mapping_node/elevation_map` | `grid_map_msgs/GridMap` | `odom` | layers `elevation`, `variance`, `elevation_filtered`; timestamp = processing time |
| `~/elevation_cloud` = `/terrain_mapping_node/elevation_cloud` | `sensor_msgs/PointCloud2` | `odom` | one point $(x, y, h_{filtered})$ per valid cell — RViz display without `grid_map_rviz_plugin` (not installed); this is the `elevation_filtered` layer, the one used for the controller's foothold heightmaps |

### 2.3 Transforms (TF) used

| Transform | Published by | Use |
|---|---|---|
| `odom → trunk` (`base_frame`) | `spawn_chien/scripts/odom_to_tf.py`, from `~/odom` of `CheetahController` | robot x, y (recentering) and trunk height z (plausibility filter) |
| `odom → camera_link_optical` | `odom → trunk` chain + `robot_state_publisher` | cloud transform, at the cloud timestamp (0.2 s wait), otherwise the latest one |
| `odom → leg{1..4}_lower`, `odom → leg{1..4}_foot` | `robot_state_publisher` (from `/joint_states`) | leg capsules of the self-filter |

### 2.4 Parameters

| Parameter | Default | Unit | Role |
|---|---|---|---|
| `map_frame` | `odom` | — | map frame (the controller's localization-corrected frame) |
| `base_frame` | `trunk` | — | robot frame for recentering and plausibility |
| `resolution` | `0.02` | m | cell size |
| `length_x`, `length_y` | `4.0`, `2.0` | m | map size (200 × 100 cells) |
| `process_noise_per_s` | `1e-5` | m²/s | Kalman process noise $q$ (variance growth) |
| `measurement_noise` | `4e-4` | m² | measurement noise $\sigma_m^2$ ((2 cm)²) |
| `max_valid_range_m` | `3.0` | m | max accepted depth (camera frame) |
| `max_plausible_delta_from_robot_m` | `0.5` | m | ground accepted down to this distance **below** the `trunk` frame |
| `max_plausible_above_robot_m` | `0.6` | m | ground accepted up to this distance **above** the `trunk` frame |
| `self_filter` | `true` | — | remove robot points (legs, trunk) |
| `self_filter_radius_m` | `0.05` | m | thigh and shank capsule radius |
| `foot_offset` | `[0.19206, -0.15791, -0.04]` | m | foot center in the `legN_foot` frame (as in `LegKinematics`) |
| `use_sim_time` | `true` (launch) | — | simulated clock |

## 3. Point cloud pipeline

`TerrainMappingNode::cloudCallback()`, in order:

| # | Step | Detail | Why |
|---|---|---|---|
| 1 | **Recenter** | `map_.move(trunk xy)` (`GridMap::move`): cells leaving the map are cleared, new ones are empty (NaN) | moving map: the robot can walk indefinitely; without TF (startup) the map stays where it is |
| 2 | **Prediction** | variance of every observed cell $+= q\,\Delta t$ (§4.1) | uncertainty grows on cells that are not seen again |
| 3 | **Timestamp** | `map_.setTimestamp(now)` | otherwise `toMessage()` publishes a 0 timestamp forever, and a subscriber ignoring same-stamp messages would see a frozen map |
| 4 | **Range filter** | finite points with $z_{camera} <$ `max_valid_range_m`, **in the sensor frame** | a "no return" pixel is saturated at max range (10 m) instead of being dropped; once rotated into `odom` it would look like a real measurement |
| 5 | **Transform** | `tf2::doTransform` to `odom` at the cloud timestamp | — |
| 6 | **Useful points** | finite and inside the map footprint | `pcl::VoxelGrid` sizes its bounding box before any filtering: a far point overflows its indices ("Leaf size is too small") |
| 7 | **VoxelGrid** | leaf of `resolution / 2` = 1 cm | uniform density, at most ~4 points per cell per frame |
| 8 | **Self-filter** | `isRobotPoint()`: thigh/shank capsules of the 4 legs + trunk box (§4.3) | the front legs are in the camera view; they would become obstacles |
| 9 | **Plausibility** | $z_{trunk} - 0.5 \le z \le z_{trunk} + 0.6$ (§4.4) | removes depth-camera "flying pixels" before they initialize a cell with false confidence |
| 10 | **Fusion** | scalar Kalman update in the point's cell (§4.1) | — |
| 11 | **Publish** | map copy, `convertToDefaultStartIndex()`, `processTerrain()` → `elevation_filtered` (§4.2), GridMap and cloud messages | the fusion map itself is never modified by post-processing |

## 4. Equations

### 4.1 Per-cell Kalman filter (`predictVariance`, `fuseMeasurement`)

Each cell is a scalar state, the ground height $h$, with variance $\sigma^2$; the terrain is
assumed **static** (random walk model).

**Prediction** (every cloud, for all observed cells, $\Delta t$ = time since the previous cloud):

$$
h^- = h, \qquad \sigma^{2-} = \sigma^2 + q\,\Delta t
$$

**Update** with a measurement $z$ (height of a point falling in the cell):

$$
K = \frac{\sigma^{2-}}{\sigma^{2-} + \sigma_m^2}, \qquad
h \leftarrow h^- + K\,(z - h^-), \qquad
\sigma^2 \leftarrow (1 - K)\,\sigma^{2-}
$$

**Initialization**: a never-observed cell has $\sigma^2$ = NaN; the first measurement initializes
it directly, $h = z$, $\sigma^2 = \sigma_m^2$.

Useful properties:
- $n$ consistent measurements without prediction in between: $\sigma^2 = \sigma_m^2 / n$, and the
  height is their mean.
- A cell seen only once keeps a high variance → high gain → it is corrected quickly by the next
  measurement (unlike a fixed-gain filter).
- A cell not seen for $T$ seconds: $\sigma^2(T) = \sigma^2 + qT$. With $q = 10^{-5}$ m²/s, 10 s add
  $10^{-4}$ m² (1 cm standard deviation): the hind feet land on cells seen several seconds
  earlier, which stay reliable.

*Difference from elevation_mapping*: ETH also propagates the robot pose uncertainty into the
measurement variance and discards inconsistent measurements with a Mahalanobis distance test.
Here the pose comes from the (corrected) external localization, the measurement variance is
constant, and consistency is enforced by the plausibility filter.

### 4.2 Post-processing (`processTerrain`): `elevation_filtered` layer

**Hole filling** (single pass, radius $\rho = 2$ cells, $n_{min} = 3$): a NaN cell gets the mean of
its valid neighbors if there are enough of them,

$$
\tilde h_{ij} = \frac{1}{|V_{ij}|} \sum_{(a,b)\in V_{ij}} h_{ab}
\quad \text{if } |V_{ij}| \ge n_{min}, \qquad
V_{ij} = \{(a,b) : |a-i| \le \rho,\ |b-j| \le \rho,\ h_{ab}\ \text{finite}\}
$$

Single pass: a filled value is never propagated from another filled value.

**3 × 3 median filter** on valid cells (valid neighbors only):

$$
\hat h_{ij} = \operatorname{median}\{\tilde h_{ab} : |a-i| \le 1,\ |b-j| \le 1,\ \tilde h_{ab}\ \text{finite}\}
$$

It removes an isolated outlier while **keeping a step edge** (a real structure covers at least half
of the neighborhood of an edge cell; see test `IsolatedFlyingPixelIsRemovedEdgeIsKept`).

### 4.3 Robot self-filter (`computeRobotShapes`, `isRobotPoint`)

Shapes are recomputed for every cloud from the latest TF (no shape is made up if a TF is missing:
the filter is then inactive for that cloud).

**Capsule** $(a, b, \rho)$ — for each leg, thigh: `legN_lower` origin → `legN_foot` origin
(knee); shank: knee → foot center ($T_{foot} \cdot d_{foot}$); $\rho$ = `self_filter_radius_m`.
A point $p$ is inside the capsule if

$$
t^* = \operatorname{clamp}\!\left(\frac{(p-a)\cdot(b-a)}{\|b-a\|^2},\, 0,\, 1\right), \qquad
\|p - (a + t^*(b-a))\| \le \rho .
$$

**Trunk box** (URDF collision box + 3 cm margin), center $c = (-0.055,\, 0.001,\, 0.518)$,
half extents $e = (0.322,\, 0.188,\, 0.117)$ m in the trunk frame: the point
$q = T_{odom \to trunk}\,p - c$ is inside if $\max_k (|q_k| - e_k) \le 0$.

### 4.4 Plausibility filter

With $z_{tr}$ the height of the `trunk` link origin (TF `odom → trunk`), a point is accepted if

$$
z_{tr} - \delta_{low} \;\le\; z \;\le\; z_{tr} + \delta_{high}, \qquad
\delta_{low} = 0.5\ \text{m},\ \delta_{high} = 0.6\ \text{m}.
$$

The `trunk` link origin of this URDF sits near the legs, not at the body center (spawn at
$z = -0.207$ m): when standing, the ground is **above** this frame by about the standing height.
The band is therefore asymmetric: wide below (map holes), and above bounded by standing height +
step + margin. The filter is needed because a newly exposed cell is initialized by its first
measurement, even a wrong one; with a low variance it could no longer be corrected.

## 5. Published layers and their consumers

| Layer | Content | Read by |
|---|---|---|
| `elevation` | Kalman-fused height (m, `odom` frame), NaN = never observed | `quadru_control::lookupTerrainHeight`: ground height under the nominal foothold (nearest observed cell within 3 cells) |
| `variance` | $\sigma^2$ per cell (m²), NaN = never observed | `lookupTerrainHeight` (a cell is "observed" if its variance is finite) |
| `elevation_filtered` | `elevation` hole-filled + 3 × 3 median | `quadru_control`: 13 × 13 foothold heightmaps of the IIT method (safety, convex regions), `~/elevation_cloud` |

Geometry: 4 m × 2 m, 2 cm resolution, centered on the trunk x, y at the last cloud, default start
index in the message (after `convertToDefaultStartIndex`).

The `terrain_processing` library is **shared**: the controller links
`terrain_mapping::terrain_processing` and uses the same layer names (`kLayerElevation`,
`kLayerFiltered`), which keeps the node and the controller consistent.

## 6. Files and functions

### `src/terrain_mapping_node.cpp` — executable `terrain_mapping_node`

Class `TerrainMappingNode : rclcpp::Node`.

| Function | Role |
|---|---|
| constructor | declares the parameters (§2.4), creates the map (`elevation`, `variance` at NaN, `map_frame`), the cloud subscription and the two publishers |
| `cloudCallback(msg)` | full pipeline for one cloud (§3) |
| `predictVariance(dt)` | Kalman prediction on all observed cells (§4.1) |
| `fuseMeasurement(idx, z)` | Kalman update in one cell, or initialization (§4.1) |
| `computeRobotShapes()` | leg capsules and trunk box in the map frame from TF; `nullopt` if a TF is missing (§4.3) |
| `lookupXY(frame)` | x, y of a frame in `map_frame` (logs if the TF is missing) |
| `lookupZ(frame)` | z of a frame in `map_frame` |
| `publishMap()` | copy, post-processing, GridMap and cell cloud publication |
| `main()` | `rclcpp::spin` of the node |

### `include/terrain_mapping/terrain_processing.hpp`, `src/terrain_processing.cpp` — library `terrain_processing`

Pure library (`grid_map_core` + Eigen, no ROS), exported (`terrain_mapping::terrain_processing`)
and unit tested.

| Element | Role |
|---|---|
| `kLayerElevation`, `kLayerFiltered` | layer names (`"elevation"`, `"elevation_filtered"`) |
| `TerrainProcessingParams` | `inpaintRadiusCells` = 2, `inpaintMinNeighbors` = 3 |
| `processTerrain(map, params)` | adds/overwrites `elevation_filtered` = median(fill(`elevation`)) (§4.2) |
| `inpaint(in, radius, minNeighbors)` (internal) | single-pass hole filling |
| `median3x3(in)` (internal) | 3 × 3 median on valid cells |
| `Capsule`, `RobotShapes` | robot shapes in the map frame |
| `isRobotPoint(p, shapes)` | does the point belong to a capsule or to the trunk box (§4.3) |

### Other files

| File | Role |
|---|---|
| `launch/terrain_mapping.launch.py` | starts the node alone (`use_sim_time: true`); the camera must already be publishing |
| `CMakeLists.txt` | library `terrain_processing` (exported), executable `terrain_mapping_node` (PCL `common`/`filters`, `pcl_conversions`, tf2), gtest |
| `package.xml` | dependencies: `rclcpp`, `sensor_msgs`, tf2, PCL, `grid_map_core/ros/msgs` |

## 7. Tests

```bash
colcon test --base-paths src --packages-select terrain_mapping && colcon test-result --verbose
```

| Test (`test/test_terrain_processing.cpp`) | Checks |
|---|---|
| `HoleIsInpainted` | a 3 × 3 hole on flat ground is filled at the right height |
| `IsolatedFlyingPixelIsRemovedEdgeIsKept` | an outlier (+30 cm) is removed by the median, the edge of a 5 cm step is kept on both sides |
| `RobotSelfFilterRejectsLimbAndTrunkPointsOnly` | points on the shank and inside the trunk are rejected; the ground ahead of the foot and a point next to the leg are kept |

## 8. Run and visualize

Normally started by the full simulation, which also enables the controller's `terrain_aware`
parameter (a single switch):

```bash
ros2 launch spawn_chien spawn.launch.py terrain_aware:=true \
  world:=$PWD/install/spawn_chien/share/spawn_chien/config/staircase_10cm.world
```

Standalone (simulation and controller already running):

```bash
ros2 launch terrain_mapping terrain_mapping.launch.py
ros2 run terrain_mapping terrain_mapping_node --ros-args -p use_sim_time:=true -p resolution:=0.03
```

Checks:

```bash
ros2 topic hz /terrain_mapping_node/elevation_map                      # ~30 Hz
ros2 topic echo /terrain_mapping_node/elevation_map --field layers     # elevation, variance, elevation_filtered
ros2 run tf2_ros tf2_echo odom trunk                                   # TF needed for recentering
```

RViz (`spawn_chien/config/terrain_camera.rviz`): `PointCloud2` on
`/terrain_mapping_node/elevation_cloud`, fixed frame `odom`. Every 5 s the node logs how many
points the self-filter removed ("TF ok" / "TF indisponible, inactif").

## 9. Design choices and limitations

- **2.5D map**: one height per cell, no overhangs (a table would be a solid block). Enough for
  stairs and rough ground.
- **Static terrain**: a moving object stays in the map until new measurements correct it (the
  Kalman gain gets smaller the more often a cell has been seen).
- **Depends on localization**: the map is built in `odom` corrected by the controller's external
  localization. Pose drift shifts new measurements relative to old ones (pose variance is not
  propagated, see §4.1).
- **Ground under the robot**: hidden from the camera by the body; cells under and behind the robot
  are no longer updated (their variance grows slowly).
- **Cost**: all processing is in the callback; 200 × 100 cells, post-processing in
  $O(N \cdot (2\rho+1)^2)$ per publication.
