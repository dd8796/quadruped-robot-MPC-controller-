# quadru_ws — Robot locomotion with ROS 2 / Gazebo

This workspace makes the robot walk in Gazebo (and, eventually, on the real robot) with **a single
controller**: `mevius_mpc_controller` = `quadru_control/CheetahController`, convex MPC locomotion
with terrain perception from an RGBD camera.

**Status (2026-10-11)**: full course validated in Gazebo (`parcours_10cm.world`: ascent
3 × 10 cm, landing, descent 3 × 10 cm, 0–4 cm tiles, 3–6 cm bumps), stairs of 5, 7.5 and
10 cm. Measurements and run history: [`runs_iit/JOURNAL.md`](runs_iit/JOURNAL.md).
Localization is still provided by Gazebo ground truth (see §7).

---

## 1. Data flow

```
 Gazebo ──► /chien/imu ───────────────────────────────┐
        ──► joint positions / velocities ─────────────┤
        ──► /chien/camera/points ──► terrain_mapping ─┼─► elevation map
        ──► /chien/ground_truth_odom (localization) ──┤
 /cmd_vel (optional) ─────────────────────────────────┤
                                                      ▼
                       CheetahController::update()  (1 kHz, quadru_control)
                         ├─ state estimator (+ REP-105 localization correction)
                         ├─ IIT foothold adaptation (planning thread, elevation map)
                         ├─ body height / pitch from the ground under the footholds
                         ├─ convex MPC (qpOASES) → ground forces of the stance legs
                         └─ swing legs: operational-space control (Khatib)
                                                      ▼
                         joint torques (effort interface, 12 joints)
```

## 2. Packages

| Package | Role |
|---|---|
| **`cheetah_locomotion`** | Locomotion library (convex MPC, state estimator, swing trajectories, gait). Local changes are marked `QUADRU:`; origin and license in [`NOTICE.md`](src/cheetah_locomotion/NOTICE.md). |
| **[`quadru_control`](src/quadru_control/README.md)** | The `ros2_control` plugin `CheetahController`: robot description (`robot_model`, `leg_kinematics`, `floating_base_model`: mass and composite inertia, leg kinematics, CRBA dynamics for the OSC), state machine, standing controller, perception, localization. Self-contained: it does not depend on the `quadru_cpp` tree (the root project builds its robot description). |
| **[`terrain_mapping`](src/terrain_mapping/README.md)** | Robot-centric elevation map (per-cell Kalman fusion, robot self-filter, inpainting, median), published on `/terrain_mapping_node/elevation_map`. |
| **`spawn_chien`** | The robot in Gazebo: URDF, meshes, `HomePosePlugin` (holds the start pose until the controller is active), worlds (stairs, courses), launch files, RViz, SLAM/Nav2. |

## 3. The `CheetahController` controller

Full documentation (topics, equations, files): [`quadru_control/README.md`](src/quadru_control/README.md).

### Hardware interfaces (`leg{1..4}_{abduction,flex,knee}`)

| Type | Name | Direction |
|---|---|---|
| State | `<joint>/position`, `<joint>/velocity` | read |
| Command | `<joint>/effort` | written (torque, N·m, saturated at ±35) |

### Subscribed topics

| Topic | Type | Use |
|---|---|---|
| `/chien/imu` | `sensor_msgs/Imu` | orientation, angular velocity, acceleration |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x/y`, `angular.z` command (heading held if `angular.z = 0`). No message received: walk straight at `walk_speed`; message older than 50 ms: stop |
| `localization_topic` | `nav_msgs/Odometry` | trunk pose (Gazebo: `/chien/ground_truth_odom`) |
| `/terrain_mapping_node/elevation_map` | `grid_map_msgs/GridMap` | elevation map (if `terrain_aware`) |
| `/chien/leg{1..4}_foot_contact` | `gazebo_msgs/ContactsState` | diagnostics only (the real robot has no contact sensors) |

### Published topics (`/mevius_mpc_controller/...`)

| Topic | Content |
|---|---|
| `~/odom` | `trunk` link pose + velocities (republished as TF `odom → trunk` by `odom_to_tf.py`) |
| `~/foothold`, `~/foothold_region` | last frozen foothold and its convex region (odom frame) |
| `~/diag_feet` (16) | foot center setpoint (x, y, z world × 4), swing (1) / stance (0) × 4 |
| `~/diag_contact` (12) | `[planned×4, planned×4, Gazebo sensor×4]` (0/1, URDF order) |
| `~/diag_force` (8) | `[commanded Fz×4, 0×4]` (N) |
| `~/diag_torques` (12) | commanded torques (N·m) |
| `~/diag_watchdog` (3) | `[tilt_rad, height_error_m, state]` — state `0` = Stabilize, `1` = Locomotion, `2` = Fault |
| `~/diag_mpc` (4) | `[ok, QP_time_ms, 0, 0]` |

### Parameters (`spawn_chien/config/mevius_controllers.yaml`, stairs profile)

The flat trot profile (`trot_flat.yaml`) and the parameters it adds (`cmd_accel_limit`,
`vel_integral_gain`, `swing_height`, `joint_damping_comp`, `joint_friction_comp`,
`foot_rolling_comp`) are described in [`quadru_control/README.md`](src/quadru_control/README.md) §2.4 and §11.

| Parameter | Value | Role |
|---|---|---|
| `localization_topic` | `/chien/ground_truth_odom` | external localization; empty = leg odometry only |
| `use_localization` | `true` | disables localization without changing the configuration |
| `gait_cycle_s`, `gait_duty` | `0.65`, `0.8` | gait: 144 ms swing (read at configure time) |
| `walk_speed` | `0.3` | m/s, speed without `cmd_vel` |
| `swing_control` | `osc` | swing legs: `osc` (operational space) or `mit` (plain Cartesian spring) |
| `swing_omega`, `swing_zeta` | `50`, `1` | bandwidth / damping of the swing control |
| `terrain_aware` | set by the launch file | use the elevation map |
| `foothold_adaptation` | `convex_mpc` | `nominal` (no adaptation), `heuristic` (nearest safe pixel), `convex_mpc` (nearest safe convex region, arXiv 2307.14775) |

### State machine

```
on_activate() ──► Stabilize ──(2.5 s)──► Locomotion
                      └───────(|roll| or |pitch| ≥ 0.5 rad)──► Fault (stand, latched)
```

- **Stabilize**: IMU wait (0.1 s), capture of the start pose and rigid-body parameters (mass,
  composite inertia, hips, nominal footholds, max force), then standing (`StandController`:
  statics τ = G(q) − Jᵀf, total weight shared by the stance feet, plus joint PD
  `kStandKp = 300`, `kStandKd = 15`) while the estimator converges.
- **Locomotion**: full loop (estimator, MPC, swing trajectories) + perception.
- **Fault**: orientation guard; the robot keeps standing, following the measured orientation.
- An abnormal `period` (> 50 ms or ≤ 0) is never integrated: the robot stands for that tick
  (test `test_update_warmup_dt_guard`).

## 4. Build and test

```bash
cd quadru_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src
colcon test --base-paths src && colcon test-result
source install/setup.bash
```

`--base-paths src` is required: `vendor/pointcloud_to_laserscan` (a `.deb` package extracted for
SLAM, see `slam.launch.py`) is not a buildable source package. Nothing is downloaded at build
time.

## 5. Run

```bash
# Flat ground
ros2 launch spawn_chien spawn.launch.py

# Stairs or course, with terrain perception
ros2 launch spawn_chien spawn.launch.py terrain_aware:=true \
  world:=$PWD/install/spawn_chien/share/spawn_chien/config/parcours_10cm.world

# Fast trot on flat ground, 0.8 m/s (speed set in config/trot_flat.yaml or with speed:=)
ros2 launch spawn_chien trot_flat.launch.py speed:=0.8
```

Arguments: `gui` (show Gazebo, `true`), `rviz` (`true`: point cloud, camera image, elevation map),
`world`, `terrain_aware` (`false`), `controllers` (controller parameter file, default
`config/mevius_controllers.yaml`), start pose `x y z roll pitch yaw`.
`trot_flat.launch.py`: `speed` (m/s, default from `trot_flat.yaml`), `gui` (`true`), `rviz` (`false`).
Worlds (`spawn_chien/config/`): `staircase` (5 cm), `staircase_7_5cm`, `staircase_10cm`,
`parcours_7_5cm`, `parcours_10cm` (generated by `scripts/make_course.py`), `corridor`,
`warehouse_*` (SLAM/Nav2).

Check the state: `ros2 topic echo /mevius_mpc_controller/diag_watchdog` (3rd value `1.0`:
locomotion active).

**Measured runs**: `runs_iit/run_mode.sh <mode> <duration_s>` (launches, records, cleans up);
variables `WORLD`, `TERRAIN_AWARE`, `LOCALIZATION`, `SWING_CONTROL`, `GAIT_CYCLE`, `GAIT_DUTY`,
`WALK_SPEED`, `GUI=true`, `BAG=1`. Analysis: `analyze_feet.py`, `course_stats.py`,
`analyze_map.py`, `summarize.py` (same folder).

**Cleanup**: `Ctrl+C` does not always kill `gzserver`, `gzclient` and the `spawner`s; a leftover
`controller_manager` makes the next launch fail (`A controller named 'joint_state_broadcaster' was
already loaded`). Before relaunching:

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f "ros2 launch spawn_chien"
pkill -9 -f spawner; pkill -9 -f robot_state_publisher; pkill -9 -x rviz2; sleep 2
ps aux | grep -E "gzserver|gzclient|ros2 launch" | grep -v grep   # must be empty
```

SLAM / Nav2: `slam.launch.py` then `nav2.launch.py`, after `spawn.launch.py` (the `odom → trunk`
TF comes from `~/odom`).

## 6. History

The previous controllers (`MpcController`: ESKF + native OSQP MPC; `mpc_control.py`; IK trot
`trot_gait.py`) were removed on 2026-10-11 and remain in the git history. Their prototyping blocks
(ESKF, OSQP MPC, gait scheduler) stay in the root project `quadru_cpp/common/` (tests, PyBullet),
outside this workspace.

## 7. Known limitations

- **Localization**: provided by Gazebo ground truth. The real robot needs a visual(-inertial)
  odometry publishing the trunk pose on `localization_topic`.
- **Pitch**: up to ~25° on 10 cm stairs, with a safety limit at 28.6° (0.5 rad).
- **Torque limit**: 35 N·m in simulation (LA8308 KV90 + 9:1 at ~42 A, peak). The continuous torque
  of the real actuators (heating, no propeller airflow) and their friction must be measured.
- **Joint speed**: the URDF velocity limit (10 rad/s) is not enforced by Gazebo; the real limit
  depends on the battery voltage (no-load ~23 rad/s at the joint in 6S, ~46 rad/s in 12S).
- **Camera**: the simulated D435 sees the robot's own hips; robot points are filtered by
  `terrain_mapping` (leg capsules + trunk box).
- **Footholds**: the MPC does not optimize foothold positions (difference from the IIT paper); the
  selected foothold is the point of the safe region nearest to the nominal foothold.
