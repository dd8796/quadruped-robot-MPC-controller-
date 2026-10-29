# quadru_control — `CheetahController` locomotion controller

`ros2_control` plugin (`controller_interface::ControllerInterface`) that makes the robot walk:
convex MPC locomotion (state estimator, convex MPC, swing trajectories, gait — library
[`cheetah_locomotion`](../cheetah_locomotion)), **visual foothold adaptation** (IIT method,
Omar et al. 2023, arXiv 2307.14775) on the elevation map of
[`terrain_mapping`](../terrain_mapping/README.md), and **external localization** (REP-105).

This package also holds the **robot description** (kinematics, mass properties, dynamics): it
depends on nothing outside `quadru_ws` (the root `quadru_cpp` project builds the same sources for
its tests and Python bindings).

In `spawn_chien` the controller is loaded as **`mevius_mpc_controller`**
(`config/mevius_controllers.yaml`), so every private topic `~/...` below is
`/mevius_mpc_controller/...`.

---

## Contents

1. [Overview and ROS 2 data flow](#1-overview-and-ros-2-data-flow)
2. [ROS 2 interfaces (topics, hardware interfaces, parameters)](#2-ros-2-interfaces)
3. [Frames and conventions](#3-frames-and-conventions)
4. [The `update()` loop step by step](#4-the-update-loop-step-by-step)
5. [State machine](#5-state-machine)
6. [Models and equations](#6-models-and-equations)
   - 6.1 Leg kinematics · 6.2 Leg gravity · 6.3 Composite inertia ·
     6.4 Floating-base dynamics (CRBA/RNEA) · 6.5 Rigid-body parameters ·
     6.6 State estimator · 6.7 External localization · 6.8 Gait · 6.9 Foot placement ·
     6.10 Convex MPC · 6.11 Swing trajectory · 6.12 Joint torques ·
     6.13 Standing controller · 6.14 Perception: ground height and pitch
7. [Visual foothold adaptation (IIT)](#7-visual-foothold-adaptation-iit)
8. [Realtime and threads](#8-realtime-and-threads)
9. [Files and functions](#9-files-and-functions)
10. [Tests](#10-tests)
11. [Build, run, debug](#11-build-run-debug)
12. [Known limitations](#12-known-limitations)

---

## 1. Overview and ROS 2 data flow

```
                          ┌───────────────────────── Gazebo / robot ─────────────────────────┐
                          │  12 joints (position, velocity)              torques (effort) ▲  │
                          └──────────┬───────────────────────────────────────────────────┼──┘
  /chien/imu (Imu, 1 kHz) ───────────┤                                                   │
  /chien/ground_truth_odom (100 Hz) ─┤ localization_topic                                │
  /cmd_vel (Twist, optional) ────────┤                                                   │
  /terrain_mapping_node/elevation_map┤ (GridMap, ~30 Hz, if terrain_aware)               │
  /chien/legN_foot_contact (diag) ───┤                                                   │
                                     ▼                                                   │
  ┌──────────────────────── CheetahController (1 kHz, realtime thread) ──────────────────┴─┐
  │ encoders + IMU ─► StateEstimator ─► localization correction (REP-105)                   │
  │                                   │                                                     │
  │         map ─► MitFootholdAdapter ─┤◄── results ── PerceptivePlanner (non-RT thread)    │
  │        (ground height, pitch)     │      requests ─►  II-A safety + II-C regions        │
  │                                   ▼                                                     │
  │      ConvexMPCLocomotion: gait, footholds, MPC (qpOASES, ~28 Hz), swing (Bezier)        │
  │                                   ▼                                                     │
  │             legTorques: stance = Jᵀ f_MPC ; swing = OSC (Khatib) or Cartesian spring    │
  └──────────────────────────────────────┬──────────────────────────────────────────────────┘
                                         ▼
     /mevius_mpc_controller/odom ──► odom_to_tf.py ──► TF odom → trunk ──► terrain_mapping
     /mevius_mpc_controller/foothold, foothold_region, diag_*   (RViz, analysis)
```

**Closed perception loop**: the controller publishes `~/odom`; `spawn_chien/scripts/odom_to_tf.py`
republishes it as the `odom → trunk` TF; `terrain_mapping` uses it to place the camera cloud in
`odom` and publishes the map; the controller reads the map in that same frame to choose its
footholds.

## 2. ROS 2 interfaces

### 2.1 Hardware interfaces (`ros2_control`)

Joints, in this order (URDF order = leg order of the locomotion library, see §3):
`leg{1..4}_abduction`, `leg{1..4}_flex`, `leg{1..4}_knee`.

| Type | Interfaces | Direction | Unit |
|---|---|---|---|
| State | `<joint>/position` ×12, then `<joint>/velocity` ×12 | read every tick | rad, rad/s |
| Command | `<joint>/effort` ×12 | written every tick | N·m, saturated at ±35 (`kTauLimit`) |

### 2.2 Subscribed topics

| Topic | Type | QoS | Rate (Gazebo) | Callback → storage | Use |
|---|---|---|---|---|---|
| `/chien/imu` | `sensor_msgs/Imu` | SensorData | 1000 Hz | `imuBuffer_` (`RealtimeBuffer`) | orientation, gyro, accelerometer for the estimator |
| `localization_topic` (parameter; `/chien/ground_truth_odom` in Gazebo) | `nav_msgs/Odometry` | SensorData | 100 Hz | `localizationBuffer_` | `trunk` link pose (world) → position correction (§6.7) |
| `/cmd_vel` (relative name `cmd_vel`) | `geometry_msgs/Twist` | SystemDefaults | any | `cmdVelBuffer_` | `linear.x`, `linear.y`, `angular.z` (body frame) |
| `/terrain_mapping_node/elevation_map` | `grid_map_msgs/GridMap` | SensorData | ~30 Hz | converted to `grid_map::GridMap`, then `PerceptivePlanner::setTerrainMap()` | elevation map (§7) |
| `/chien/leg{1..4}_foot_contact` | `gazebo_msgs/ContactsState` | SensorData | 200 Hz | `ContactSensing` | **diagnostics only**: locomotion does not use it (the real robot has no foot sensors) |

`cmd_vel` behaviour (`CheetahController::command()`):
- no message ever received → walk straight at `walk_speed` (unattended runs);
- last message older than 50 ms (`kSensorTimeoutNs`) → zero command (stepping in place);
- `angular.z = 0` → heading hold (§6.10, frozen yaw setpoint).

Command shaping (both off by default, used by the flat trot profile):
- **acceleration limit** `cmd_accel_limit` $a_{max}$: the command moves towards its target by at
  most $a_{max}\,\Delta t$ per tick (ramp from standstill and on speed changes);
- **speed integral** `vel_integral_gain` $k_i$: once the command is steady,
  $I \mathrel{+}= k_i\,\Delta t\,(v_{cmd} - \hat v)$, bounded at ±0.2 m/s, and $v_{cmd} + I$ is sent
  to the MPC. It removes the steady-state speed error of the MPC (unmodeled drag), like the
  pitch/roll/height integrals of §6.10. $\hat v$ must be unbiased: use it with `foot_rolling_comp`.

### 2.3 Published topics

All are published from the realtime loop with `realtime_tools::RealtimePublisher`
(`trylock()`: a message is skipped rather than blocking the loop).

| Topic | Type | Content | When |
|---|---|---|---|
| `~/odom` | `nav_msgs/Odometry` | `frame_id = odom`, `child_frame_id = trunk`; **`trunk` link origin** (not the CoM); linear and angular velocity **in the trunk frame** | every tick after initialization |
| `~/foothold` | `geometry_msgs/PointStamped` | frozen foothold (foot center, `odom` frame) | when a foothold is frozen (mid-swing), `terrain_aware` |
| `~/foothold_region` | `geometry_msgs/PolygonStamped` | selected convex rectangle (4 corners at ground height) | same, `convex_mpc` mode |
| `~/diag_feet` | `std_msgs/Float64MultiArray` (16) | foot center setpoint (x, y, z world × 4, URDF order), then swing (1) / stance (0) × 4 | Locomotion |
| `~/diag_contact` | `Float64MultiArray` (12) | `[planned ×4, planned ×4, raw Gazebo sensor ×4]` (0/1) | every tick |
| `~/diag_force` | `Float64MultiArray` (8) | `[MPC commanded F_z ×4 (N), 0 ×4]` | every tick |
| `~/diag_torques` | `Float64MultiArray` (12) | commanded torques (N·m) | every tick |
| `~/diag_watchdog` | `Float64MultiArray` (3) | `[tilt (rad), height error (m), state]` (height error = absolute difference between the estimated CoM height and the nominal body height, ground height not included); state 0 = Stabilize, 1 = Locomotion, 2 = Fault | every tick |
| `~/diag_mpc` | `Float64MultiArray` (4) | `[QP solved (1/0), QP time (ms), 0, 0]` | every tick |
| `~/diag_control` | `Float64MultiArray` (131) | setpoints next to measurements, `layout.dim[0].label` = `t,tau[12],q[12],qd[12],p_des[12],p[12],v_des[12],v[12],swing[4],f_mpc[12],x_ref[12],x_est[12],v_cmd[3],mpc[3]` (feet hip-relative, MPC body frame; `f_mpc`, `x_ref`, `x_est` in the MPC world frame, state order roll, pitch, yaw, x, y, z, ω, v) | Locomotion, 500 Hz |

### 2.4 Parameters

| Parameter | Type | Default | Read | Role |
|---|---|---|---|---|
| `localization_topic` | string | `""` | configure | external localization topic; empty = leg odometry only |
| `use_localization` | bool | `true` | live | disables the localization correction without reconfiguring |
| `terrain_aware` | bool | `false` | live | use the elevation map (set to `true` by `spawn.launch.py terrain_aware:=true`) |
| `foothold_adaptation` | string | `convex_mpc` | live | `nominal` (no adaptation), `heuristic` (nearest safe pixel, ref. [8] of the paper), `convex_mpc` (nearest safe convex region, IIT paper) |
| `swing_control` | string | `osc` | live | swing legs: `osc` (operational space) or `mit` (plain Cartesian spring) |
| `swing_omega` | double | `50.0` | live | natural frequency ω (rad/s) of the OSC swing control |
| `swing_zeta` | double | `1.0` | live | damping ratio ζ of the OSC swing control |
| `walk_speed` | double | `0.3` | live | speed (m/s) when no `cmd_vel` has been received |
| `cmd_accel_limit` | double | `0.0` | live | command acceleration limit (m/s²), 0 = off (§2.2) |
| `vel_integral_gain` | double | `0.0` | live | speed integral gain (1/s), 0 = off (§2.2) |
| `swing_height` | double | `0.08` | configure | swing apex height above the higher of the two footholds (m) |
| `joint_damping_comp` | double | `0.0` | live | joint viscous friction feedforward $b$ (N·m·s/rad, §6.12) |
| `joint_friction_comp` | double | `0.0` | live | joint Coulomb friction feedforward $f$ (N·m, §6.12) |
| `foot_rolling_comp` | bool | `false` | live | leg odometry at the contact point of the rolling foot (§6.6) |
| `gait_cycle_s` | double | `0.65` | configure | gait cycle duration (s) |
| `gait_duty` | double | `0.8` | configure | stance fraction (0.8 → 144 ms swing) |

"Live": can be changed with `ros2 param set` while walking (`add_on_set_parameters_callback`,
values held in `std::atomic`).

## 3. Frames and conventions

| Frame | Definition | Used by |
|---|---|---|
| **world / `odom`** | z up, flat ground at z = 0; same as the Gazebo world when localization is ground truth | map, `~/odom`, published footholds |
| **URDF trunk (`trunk`)** | `trunk` link of `urdf_chien.urdf`; **the robot's forward direction is −x**; its origin is near the hips, its CoM at `kTrunkCom = (−0.052, 0.0013, 0.510)` | kinematics (`LegKinematics`), dynamics |
| **MPC body** | x forward, y left, z up; origin = **whole-body CoM** at the start pose | estimator, MPC, trajectories |
| **MPC world** | world in x, y; $z_{MPC} = z_{world} - r_{foot}$ (z = 0 at the foot centers on flat ground) | estimator, MPC |

URDF trunk ↔ MPC body: rotation of π about z,

$$
R_f = \mathrm{diag}(-1, -1, 1) = R_f^\top = R_f^{-1}, \qquad v_{trunk} = R_f\, v_{MPC} .
$$

Orientation: $R_{world \leftarrow MPC} = R_{world \leftarrow trunk}\, R_f$. Legs: URDF 1..4 = library
legs 0..3 = front right, front left, hind right, hind left (same order, no permutation). Foot
contact sphere radius: $r = 0.02$ m (`kFootRadius`).

Position conversions used in the code:

$$
p^{global\,CoM}_{MPC} = p^{trunk}_{world} + R_{wt}\, c_{trunk} - (0, 0, r)^\top
\quad\text{(localization → MPC, `applyLocalization`)}
$$

$$
p^{trunk\,link\,CoM}_{world} = p_{MPC} + (0,0,r)^\top + R\,(k_{TrunkCom} - c_{trunk})
\quad\text{(MPC → trunk, `trunkState`)}
$$

$$
p^{trunk\,origin}_{world} = p^{trunk\,link\,CoM}_{world} - R\, k_{TrunkCom}
\quad\text{(`publishOdom`)}
$$

where $c_{trunk}$ = whole-body CoM in the trunk frame at the start pose (`comTrunk_`).

## 4. The `update()` loop step by step

`CheetahController::update(time, period)` is called by the `controller_manager` at 1000 Hz
(`update_rate`, Gazebo physics step = 1 ms).

1. **Read** the 12 positions $q$ and 12 velocities $\dot q$; `contacts_.read()` (diagnostics).
2. First call: the current pose becomes the standing target (`stand_->setTarget(q)`).
3. **Guards**: Fault state, or $\Delta t \le 0$ or $\Delta t > 50$ ms, or no IMU yet → stand with
   the IMU orientation and return (an abnormal period is never integrated by the estimator).
4. **Initialization** (once, after 0.1 s of valid IMU):
   - `buildRobotParams(q, R_imu)`: mass, composite inertia, hips, nominal footholds, CoM height,
     max force (§6.5);
   - `ConvexMPCLocomotion(dt, iterations, robot)` with
     $\text{iterations} = \mathrm{round}\!\left(\frac{T_{cycle}}{18\,\Delta t}\right)$ = 36;
   - `StateEstimator` reset at the start pose.
5. **Estimation**: `readLegs` (FK + Jacobians in the MPC body frame), `readImu`, then
   `estimator_->run()` (§6.6) and `applyLocalization()` (§6.7).
6. **Safety**: $|\phi| \ge 0.5$ rad or $|\theta| \ge 0.5$ rad → **Fault** (latched until
   re-activation) and stand.
7. **Stabilize** (2.5 s): standing controller (§6.13), contact phase 0.5 on all 4 legs (full trust
   in the estimator), then `locomotion_->initialize()` → Locomotion.
8. **Locomotion**:
   - `updatePerception(q)`: map for this tick, foothold adapter context, ground height under the
     feet → estimator and height reference, terrain pitch (§6.14);
   - `locomotion_->run(se, legs, command(), legCmds_, contactPhase_)`: gait, footholds, MPC (one
     QP every 36 iterations), swing trajectories, leg commands;
   - publish the frozen footholds;
   - `legTorques()` (§6.12), then write the torques.
9. **Publish** (`publish`): `~/diag_feet`, `~/odom`, diagnostics.

## 5. State machine

```
 on_activate() ─► Stabilize ──(t ≥ 2.5 s)──► Locomotion
                     │                           │
                     └───(|roll| or |pitch| ≥ 0.5 rad)───► Fault (stand, latched)
```

| State | Command | Exit |
|---|---|---|
| IMU wait (< 0.1 s) | stand, identity orientation | valid IMU (finite quaternion, norm > 0.5) |
| **Stabilize** (0) | `StandController` | after `kStabilizeDuration` = 2.5 s |
| **Locomotion** (1) | MPC + swing + perception | Fault if tilt exceeds 0.5 rad |
| **Fault** (2) | stand, following the IMU orientation | `on_activate()` (controller re-activation) |

## 6. Models and equations

### 6.1 Leg kinematics (`LegKinematics`)

Each leg: 3 revolute joints in series (abduction, flex, knee) about their local z axis. For joint
$k$ the URDF gives a fixed transform $T^{fix}_k(xyz, rpy)$; with
$R(rpy) = R_z(y) R_y(p) R_x(r)$:

$$
T(q) = \prod_{k=1}^{3} T^{fix}_k\, \mathrm{Rot}_z(q_k), \qquad
p_{foot}(q) = T(q) \begin{bmatrix} d_{foot} \\ 1 \end{bmatrix}
$$

with $d_{foot} = (0.192, -0.158, -0.040)$ the contact sphere center in the last link frame.
Geometric Jacobian (trunk frame), $a_k$ axis and $o_k$ origin of joint $k$:

$$
J_{:,k} = a_k \times (p_{foot} - o_k), \qquad \dot p_{foot} = J(q)\,\dot q .
$$

**Inverse kinematics** (damped least squares, `inverseKinematics`):

$$
e = p^* - p_{foot}(q), \qquad
\Delta q = J^\top \left(J J^\top + \lambda^2 I\right)^{-1} e, \qquad \lambda^2 = 10^{-4}
$$

iterated until $\|e\| <$ tol (60 iterations max); the knee branch is the one of `qInit`.

### 6.2 Leg gravity (`gravityCompensation`)

Torque holding the leg against gravity (trunk fixed), $g_b$ = gravity in the trunk frame,
$c_j$ CoM of link $j$:

$$
\tau_k = -\sum_{j \ge k} m_j\; g_b^\top \big(a_k \times (c_j - o_k)\big) .
$$

The legs are 66 % of the robot mass (4 × 2.40 kg out of 14.58 kg): this term is not negligible.

### 6.3 Composite inertia (`RobotModel::compositeInertia`)

Total mass $M = m_{trunk} + \sum_i m_i$, global CoM
$c = \frac{1}{M}\left(m_{trunk}c_{trunk} + \sum_i m_i c_i\right)$, then the parallel axis theorem
for every body, $d_i = c_i - c$:

$$
I_c = \sum_i \left( R_i I_i^{loc} R_i^\top + m_i \left(\|d_i\|^2 I_3 - d_i d_i^\top\right) \right)
$$

returned in the MPC frame: $c \leftarrow R_f c$, $I \leftarrow R_f I R_f^\top$.
`RobotModel::trunkOnly()` gives the trunk link mass and inertia (5.00 kg) and the total mass
(14.58 kg).

### 6.4 Floating-base dynamics (`FloatingBaseModel`, Featherstone)

Generalized velocity $\nu = [\omega_b;\, v_b;\, \dot q_{1..12}] \in \mathbb{R}^{18}$ (base in the
trunk frame, free joint $S_0 = I_6$). 13 bodies (trunk + 3 × 4 links).

- Spatial inertia of a body (mass $m$, CoM $c$, inertia $I_c$):
  $\; \mathcal{I} = \begin{bmatrix} I_c + m [c]_\times [c]_\times^\top & m[c]_\times \\ m[c]_\times^\top & m I_3 \end{bmatrix}$
- Plücker transform parent → child (rotation $E$, origin $r$):
  $\; X = \begin{bmatrix} E & 0 \\ -E[r]_\times & E \end{bmatrix}$
- **CRBA** (mass matrix, `massMatrix`): $\mathcal{I}^C_{\lambda(i)} \mathrel{+}= X_i^\top \mathcal{I}^C_i X_i$
  from leaves to root, then $M_{ij} = S_i^\top X^\top \cdots \mathcal{I}^C_i S_j$ walking up the
  chain. $M(q)$ depends on the joint angles only.
- **RNEA** (`inverseDynamics`): $v_i = X_i v_{\lambda(i)} + S_i \dot q_i$,
  $a_i = X_i a_{\lambda(i)} + S_i \ddot q_i + v_i \times S_i \dot q_i$,
  $f_i = \mathcal{I}_i a_i + v_i \times^* \mathcal{I}_i v_i$, then $f_{\lambda(i)} \mathrel{+}= X_i^\top f_i$,
  $\tau_i = S_i^\top f_i$. Gravity enters as a base acceleration ($a_0 \mathrel{-}= R_0^\top g$).
- `biasForces` = $h(q,\nu) = C\nu + g$ = RNEA with $\ddot q = 0$; `contactJacobian`,
  `footBiasAcceleration`: foot contact Jacobian and $\dot J \nu$ term.

**Use in the controller**: only the $3\times3$ block of $M(q)$ of each leg is used, for the
operational-space inertia of the swing control (§6.12). CRBA/RNEA consistency is tested in the
root project (`test/test_floating_base_model.cpp`).

### 6.5 Rigid-body parameters (`buildRobotParams`)

At the start pose $q_0$ and orientation $R_0$:

| Quantity | Computation |
|---|---|
| mass | $M$ = total mass (14.58 kg) |
| inertia | composite $I_c$ about the global CoM (§6.3), MPC frame |
| hip $i$ | $R_f\,(o^{abd}_i - c_{trunk})$ |
| nominal foothold $i$ | $R_f\,(p_{foot,i}(q_0) - c_{trunk})$ (z ignored) |
| body height | $h = -\min_i \big(R_0 (p_{foot,i} - c_{trunk})\big)_z$: CoM above the plane of the foot centers |
| max force | $F_{max} = \min_i \dfrac{\tau_{max}}{\max_k \left\lvert\left(J_i^\top R_0^\top e_z\right)_k\right\rvert}$: vertical force that saturates the first motor |
| swing height | `swing_height` (0.08 m stairs, 0.06 m flat trot) |
| gait | 18 segments, offsets (0, 9, 9, 0), stance $\mathrm{round}(0.8 \times 18) = 14$ segments |

### 6.6 State estimator (`cheetah::StateEstimator`)

**Orientation**: taken from the IMU filter, converted to the MPC body frame;
$\omega_{world} = R^\top \omega_{body}$, $a_{world} = R^\top a_{body}$. The initial yaw is kept: the
world frame must stay the map frame.

**Position / velocity**: linear Kalman filter. State
$\hat x = [p;\, v;\, p_{foot,1..4}] \in \mathbb{R}^{18}$ (MPC world).

Prediction ($a = a_{world} + g$):

$$
\hat x^- = A \hat x + B a, \quad
A = \begin{bmatrix} I & \Delta t I & 0 \\ 0 & I & 0 \\ 0 & 0 & I_{12} \end{bmatrix},\;
B = \begin{bmatrix} 0 \\ \Delta t I \\ 0 \end{bmatrix}, \quad
P^- = A P A^\top + Q .
$$

Measurements $y \in \mathbb{R}^{28}$, for each leg $i$
($p_{f,i} = R^\top(\text{hip}_i + p_{leg,i})$,
$\dot p_{f,i} = R^\top(\omega \times (\text{hip}_i + p_{leg,i}) + \dot p_{leg,i})$):

$$
\underbrace{p - p_{foot,i}}_{\text{model}} = -p_{f,i}, \qquad
\underbrace{v}_{\text{model}} = (1-c_i)\, v^- + c_i\,(-\dot p_{f,i}), \qquad
\underbrace{z_{foot,i}}_{\text{model}} = (1-c_i)(p_z + p_{f,i,z}) + c_i\, h_{ground,i}
$$

where $c_i$ is the **contact trust**, a ramp over the planned stance phase $\varphi_i$
(window 0.2):

$$
c_i = \begin{cases} \varphi_i / 0.2 & \varphi_i < 0.2 \\ (1-\varphi_i)/0.2 & \varphi_i > 0.8 \\ 1 & \text{otherwise}\end{cases}
$$

and the noises of leg $i$ (foot process, velocity and height measurements) are multiplied by
$1 + 100\,(1 - c_i)$: a swing leg does not constrain the state. Standard update:

$$
S = C P^- C^\top + R, \quad \hat x = \hat x^- + P^- C^\top S^{-1}(y - C\hat x^-), \quad
P = (I - P^- C^\top S^{-1} C) P^- ,
$$

then $P$ is symmetrized. Noise variances: IMU process 0.02; foot process 0.002; position
measurement 0.001, velocity 0.1, height 0.001.

$h_{ground,i}$ is the ground height under the foot (§6.14); without it, on a step, the estimator
would sink the body by the step height.

**No foot sensors**: contact is the phase planned by the gait.

**Rolling foot** (`foot_rolling_comp`, `contactPointVelocities`): the foot is a 2 cm sphere that
rolls during stance, so the point at rest is its contact point, not its center. With
$d = -r\,e_z$ (contact point − center) and $\omega_{foot} = \omega_{body} + J_\omega(q)\,\dot q$
(`LegKinematics::angularJacobian`), the estimator receives the velocity of the contact point,

$$
\dot p_{contact,rel} = J(q)\,\dot q + \omega_{foot} 	imes d ,
$$

instead of the center velocity. Without it the leg odometry underestimates the speed by about
$r / L_{leg} pprox 7$ % (measured: 0.697 m/s estimated for 0.747 m/s true; with it, < 1 %).

### 6.7 External localization (`applyLocalization`, REP-105)

The estimator stays continuous; on every **new** localization sample (new timestamp) a position
offset is recomputed, like `map → odom` in REP-105:

$$
\Delta_k = p^{loc}_{MPC}(t_k) - \hat p_{KF}(t_k), \qquad
p_{used}(t) = \hat p_{KF}(t) + \Delta_{k}\quad (t \ge t_k)
$$

The orientation is the localization one on the tick it arrives, the IMU one otherwise.
Velocities stay those of the filter. The corrected pose is injected with
`StateEstimator::overridePose()`. A late or missing sample does not make the pose jump back to
the drifted leg odometry: the last offset stays applied.

### 6.8 Gait (`OffsetDurationGait`)

The cycle $T_{cycle} = 0.65$ s is split into $N = 18$ MPC segments of
$\Delta t_{MPC} = 36\,\Delta t = 36$ ms (horizon = one cycle, 648 ms). Leg $i$: offset $o_i$,
stance duration $d_i = 14$ segments. Trot: $o = (0, 9, 9, 0)$, diagonal pairs (FR+HL, FL+HR)
alternate. Swing = $4 \times 36 = 144$ ms. `getMpcTable()` gives, for each horizon step, the
planned contact $s_{j,i} \in \{0,1\}$ of each leg; `getContactState()` / `getSwingState()` give
the current phase within stance / swing (∈ [0,1], 0 outside that phase).

### 6.9 Foot placement (Raibert heuristic + capture point)

For each leg, every tick, in the MPC world frame ($T_{st}$ stance duration, $T_{rest}$ remaining
swing time, $\dot\psi$ yaw rate command, $h$ height):

$$
P_f = p + R^\top\!\left( R_z\!\left(-\dot\psi\, \tfrac{T_{st}}{2}\right) p^{nom}_i + v_{des}\, T_{rest} \right)
+ \begin{bmatrix}
\tfrac{T_{st}}{2}\, v_x + 0.03\,(v_x - v^{des}_x) + \tfrac{h}{2g}\, v_y \dot\psi \\
\tfrac{T_{st}}{2}\,\Delta t_{MPC}\, v_y + 0.03\,(v_y - v^{des}_y) - \tfrac{h}{2g}\, v_x \dot\psi \\
0
\end{bmatrix}
$$

(correction terms saturated at ±0.3 m), $P_{f,z} = -0.003$ m. With `terrain_aware`,
`MitFootholdAdapter::adapt()` then replaces $P_f$ (x, y, z) with the foothold chosen on the map
(§7).

### 6.10 Convex MPC (`SolverMPC`, Di Carlo et al. 2018, qpOASES)

**Model**: single rigid body, state $x = [\Theta;\, p;\, \omega;\, v;\, -g] \in \mathbb{R}^{13}$
(roll, pitch, yaw; position; world angular velocity; velocity; gravity), inputs
$u = [f_1; \dots; f_4] \in \mathbb{R}^{12}$ (ground forces on the feet, world). With
$r_i = p_{foot,i} - p$ and the world inertia approximated with yaw only,
$I_w = R_z(\psi)\, I_c\, R_z(\psi)^\top$:

$$
\dot\Theta \approx R_z(\psi)^\top \omega, \quad
\dot p = v, \quad
\dot\omega = I_w^{-1} \sum_i r_i \times f_i, \quad
\dot v = \frac{1}{M}\sum_i f_i + g .
$$

That is $\dot x = A_c x + B_c u$ (plus an `x_drag` term coupling $\dot v_z$ to $v_x$: integral of
the height error at non-zero speed).

**Exact discretization** (matrix exponential, `c2qp`):

$$
\exp\!\left( \Delta t_{MPC} \begin{bmatrix} A_c & B_c \\ 0 & 0 \end{bmatrix} \right)
= \begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix}
$$

**Condensed form** over the horizon $N = 18$: $X = A_{qp} x_0 + B_{qp} U$ with
$A_{qp} = [A_d; A_d^2; \dots; A_d^N]$ and $(B_{qp})_{rc} = A_d^{r-c} B_d$ for $r \ge c$.

**QP** (variables $U \in \mathbb{R}^{12N}$):

$$
\min_U\; (X - X_d)^\top S (X - X_d) + \alpha\, U^\top U
\;\Longleftrightarrow\;
\min_U\; \tfrac12 U^\top H U + U^\top g,
$$

$$
H = 2\left(B_{qp}^\top S B_{qp} + \alpha I\right), \qquad g = 2 B_{qp}^\top S \left(A_{qp} x_0 - X_d\right)
$$

subject to, for every step $j$ and leg $i$ (friction pyramid, $\mu = 0.4$):

$$
0 \le f_z \le s_{j,i}\, F_{max}, \qquad |f_x| \le \mu f_z, \qquad |f_y| \le \mu f_z .
$$

Forces of a swing leg ($s_{j,i} = 0$, zero upper bound) are **eliminated** from the QP before
solving (variables and constraints reduced). Solver: qpOASES (`QProblem`, at most 100 working set
changes).

| Weights $S$ (per step) | $\phi$ | $\theta$ | $\psi$ | $p_x$ | $p_y$ | $p_z$ | $\omega_x$ | $\omega_y$ | $\omega_z$ | $v_x$ | $v_y$ | $v_z$ |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | 0.25 | 0.25 | 10 | 2 | 2 | 50 | 0 | 0 | 0.3 | 0.2 | 0.2 | 0.1 |

$\alpha = 4\cdot10^{-5}\,(9/M)^2$: the cost $\alpha f^2$ grows with $M^2$, so $\alpha M^2$ is kept
constant across robot masses.

**Reference trajectory** $X_d$: x, y position integrated from $v^{des}_{world}$ starting from a
point within 10 cm of the measured position; $z = h + h_{ground}$; pitch
$= \theta_{terrain} + $ integral compensation; roll 0; yaw: frozen setpoint if
$\dot\psi_{des} = 0$ (**heading hold**, otherwise the heading drifts ~12° / 10 m), else
integration of $\dot\psi_{des}$. The velocity command is first-order filtered
($v \leftarrow 0.9\,v + 0.1\,v_{cmd}$ every tick).

**Output**: $f_i$ of the first horizon step; force applied by the leg (MPC body frame)
$f^{ff}_i = -R\, f_i$. The QP is solved every 36 ms (≈ 28 Hz) and the forces are held in between.

### 6.11 Swing trajectory (`FootSwingTrajectory`)

Cubic Bezier between the foot at lift-off $p_0$ and the foothold $p_f$, phase $s \in [0,1]$,
duration $T_{sw}$:

$$
b(s) = s^3 + 3s^2(1-s) = 3s^2 - 2s^3, \qquad
p_{xy}(s) = p_{0} + b(s)\,(p_f - p_0), \quad
\dot p = \frac{6s(1-s)}{T_{sw}}(p_f - p_0), \quad
\ddot p = \frac{6 - 12s}{T_{sw}^2}(p_f - p_0)
$$

In z, two half Beziers: $p_{0,z} \to z_{apex}$ for $s \in [0, 0.5]$, then $z_{apex} \to p_{f,z}$,
with

$$
z_{apex} = \max(p_{0,z},\, p_{f,z}) + h_{swing}, \qquad h_{swing} = 0.08\ \text{m}
$$

(an apex of $p_{0,z} + h_{swing}$ would only clear a step nose by $h_{swing} - $ riser when
climbing). The same curve is used for the frontal collision check (§7.3 c), with the same
parameters.

### 6.12 Joint torques (`legTorques`)

Leg command law, foot position/velocity relative to the hip, MPC body frame:

$$
\tau = J^\top\left( f^{ff} + K_p (p_{des} - p) + K_d (v_{des} - v) \right) - K_{d,q}\, \dot q
$$

| Phase | $f^{ff}$ | $K_p$ (N/m) | $K_d$ (N·s/m) | $K_{d,q}$ |
|---|---|---|---|---|
| stance | $-R f_{MPC}$ | 0 | diag(7, 7, 7) | 0.2 |
| swing, `swing_control: mit` | 0 | diag(700, 700, 150) | diag(7, 7, 7) | 0 |

**Swing, `swing_control: osc` (default)**: operational-space control (Khatib 1987); stiffness is
scaled by the actual leg inertia seen at the foot,

$$
\Lambda = \left(J\, M_{leg}^{-1} J^\top\right)^{-1}, \qquad
\tau = J^\top \Lambda \left( \ddot p_{des} + \omega^2 e + 2\zeta\omega\, \dot e \right) + G_{leg}(q),
$$

$e = p_{des} - p$, $\dot e = v_{des} - v$, $\ddot p_{des}$ = Bezier acceleration
(`LegCommand::aDes`), $M_{leg}$ = $3\times3$ block of the CRBA mass matrix, $G_{leg}$ = leg
gravity (§6.2), $\omega = 50$ rad/s, $\zeta = 1$. The plain spring ($K_p = 700$ N/m) is tuned for
~0.6 kg legs: measured here, the foot only reached 66–73 % of the swing height.

**Joint friction feedforward** (`joint_damping_comp` $b$, `joint_friction_comp` $f$, all phases):
$	au \mathrel{+}= b\,\dot q + f 	anh(\dot q / 0.05)$. The MPC does not model joint friction; at
0.8 m/s the stance joints turn at 3–5 rad/s, so 0.5 N·m·s/rad of damping brakes the robot. The
simulation uses the URDF `<dynamics>` values (0.5, 0.1); the real actuators must be identified.

All torques are saturated at ±35 N·m (LA8308 KV90 + 9:1 at ~42 A). At the start pose this gives
$F_{max} pprox 150$ N per foot (§6.5).

### 6.13 Standing controller (`StandController`)

Floating-base statics, per leg, $R$ trunk orientation, $n$ stance legs (4 in Stabilize/Fault):

$$
\tau = G(q) - J^\top R^\top f + K_p (q^* - q) - K_d \dot q, \qquad
f = \left(0, 0, \tfrac{M g}{n}\right)^\top \ \text{(stance legs, 0 otherwise)}
$$

$K_p = 300$ N·m/rad, $K_d = 15$ N·m·s/rad (same gains as the `gazebo_ros2_control` position PID
holding the robot at spawn: no stiffness gap at handover), $q^*$ = pose at the first tick.
Equivalent of the `FixedStand` state of unitree_guide.

### 6.14 Perception: ground height and pitch (`updatePerception`)

With `terrain_aware` and a map received:

- **Ground height under each foot** $h_{ground,i}$: height targeted by the leg's last swing
  (`MitFootholdAdapter::footGround()`), sent to the estimator (§6.6);
- **Body height reference**: $z_{ref} = h + \overline{h_{ground}}$ (mean of the 4 feet; the ground
  under the body itself is hidden from the camera);
- **Pitch reference** = slope of the footholds over the nominal foothold wheelbase:

$$
\theta_{terrain} = -\operatorname{atan2}\!\left(
\tfrac{h_{ground,0} + h_{ground,1}}{2} - \tfrac{h_{ground,2} + h_{ground,3}}{2},\;
\tfrac{x^{nom}_0 + x^{nom}_1}{2} - \tfrac{x^{nom}_2 + x^{nom}_3}{2}\right)
$$

(negative = nose up). The body follows the stairs instead of staying level.

Without a map: nominal footholds, flat ground ($h_{ground} = 0$, $\theta_{terrain} = 0$).

## 7. Visual foothold adaptation (IIT)

S. Omar, L. Amatucci, G. Turrisi, V. Barasuol, C. Semini, *Fast Convex Visual Foothold
Adaptation for Quadrupedal Locomotion*, IIT, arXiv 2307.14775 (2023) — PDF at the repository root.

### 7.1 One swing (`MitFootholdAdapter::adapt`, called every tick by the locomotion loop)

```
 lift-off                                        mid-swing (φ > 0.5)              touchdown
    │ request (try_lock) ─► PerceptivePlanner     │ foothold FROZEN, published     │ h_ground,i = target.z
    │   13×13 heightmap around the nominal        │ (~/foothold, ~/foothold_region)│
    │   II-A criteria → safety map                │                                │
    │   II-C convex regions                       │                                │
    │◄── result (RealtimeBuffer) a few ms later   │                                │
    │ target = current nominal clamped into the region (recomputed every tick)     │
```

1. **Nominal**: $P_f$ from §6.9; height = nearest observed cell within 3 cells (6 cm) on the
   `elevation` layer (`lookupTerrainHeight`), otherwise the ground of the leg's last foothold.
2. **Request** (once per swing): nominal position, leg angles, trunk orientation, trunk origin now
   and **predicted at touchdown** $p_{trunk}^{td} = p_{trunk} + v\,(1 - \varphi)\,T_{sw}$, foot
   position at lift-off.
3. **Until the foothold is frozen**: target = point of the selected region nearest to the
   **current** nominal (the nominal moves with the velocity), height = region height;
   `heuristic` mode: nearest safe point to the current nominal in the safety map.
4. **At mid-swing** ($\varphi > 0.5$, `kFootholdCommitPhase`): the foothold is frozen until
   touchdown.
5. Target: $P_f = (x, y, h_{ground} - 0.003)$ (MPC z = foot center plane, see §3).

If no pixel is safe, the nominal foothold is kept (and logged with the rejection count per
criterion).

### 7.2 Foothold heightmap (`extractFootholdHeightmap`)

Square window of half-size 0.12 m (`kFootholdWindowHalf`) centered on the nominal, at the map
resolution (2 cm) → **13 × 13 pixels**, layer `elevation_filtered` (hole-filled + median, see
`terrain_mapping`). NaN = unknown.

### 7.3 II-A safety criteria (`evaluateFootholdSafety`)

A pixel $(i,j)$ of height $h$, target $t = (x_{ij}, y_{ij}, h + r)$, is rejected by the **first**
failing criterion (counters in `SafetyStats`):

| Order | Criterion | Test | Parameters |
|---|---|---|---|
| 0 | unknown | $h$ = NaN | — |
| b | **roughness** | number of neighbors (radius 2 cells, unknown included) with $\lvert h_n - h\rvert > 2$ cm $\ge 2$ | `roughnessRadiusCells`, `roughnessHeightThreshold`, `roughnessMinNeighbors` |
| a | **reachability** | IK (§6.1) of $R^\top(t - p_{trunk}^{td})$ converges to 3 mm, $\lvert q_{knee}\rvert \ge 0.25$ rad, same knee sign as now | `ikTol`, `minKneeAbs` |
| c | **frontal collision** | the actual swing trajectory (Bezier §6.11, from the lift-off foot to $t$), sampled at $s = 0.9k/12$, $k = 1..11$: the foot sphere does not penetrate the terrain | `swingSamples`, `liftHeight`, `swingDuration`, `collisionTol` |
| d | **leg collision** | thigh (flex → knee) and shank (knee → 80 % of the foot) above the terrain + 1 cm, at touchdown (IK at the predicted trunk) and at $s \in \{0.25;\, 0.5;\, 0.75\}$ of the swing (interpolated trunk) | `limbSamples`, `limbClearance` |

**Sphere penetration** (`sphereHitsTerrain`), center $c$, radius $r$:

$$
h(c_x, c_y) > c_z - r + tol \quad\text{or}\quad
\exists k:\ h\!\left(c + \tfrac{r}{2}(\cos\tfrac{k\pi}{4}, \sin\tfrac{k\pi}{4})\right) > c_z - \tfrac{\sqrt3}{2} r + tol
$$

(sampled at half radius, not at the radius: a foot resting against a riser is not a collision).

Section II-B of the paper (SaFE-Net) is only a learned approximation of these criteria to speed
them up; here the window is small and the criteria are computed directly (a few ms in the planning
thread).

### 7.4 II-C convex decomposition (`decomposeConvexRegions`, `selectNearestRegion`)

The binary safety map (non-convex, possibly disconnected) is greedily split into **maximal
rectangles** (principle of the *adaptive-boxes* library):

1. largest all-ones rectangle of the matrix: column histograms of consecutive ones, row by row,
   with a monotonic stack ($O(n^2)$ per pass);
2. remove it (cells set to 0) and repeat, up to 8 regions or until the area is below 4 cells;
3. region height = **median** of its cell heights.

Selected region: minimum point-to-rectangle distance to the nominal,
$d = \|n - \mathrm{clamp}(n, r_{min}, r_{max})\|$; ties go to the largest. Foothold:
$\mathrm{clamp}(n, r_{min}, r_{max})$.

**Difference from the paper**: in the IIT method the convex region becomes an MPC constraint and the
foothold position a decision variable. Here the QP does not have footholds as variables: the
foothold is the point of the region nearest to the nominal.

### 7.5 `heuristic` mode (`nearestSafeFoothold`)

Reference [8] of the paper (Villarreal et al. 2020): nearest safe point to the nominal; in each
safe cell, the point nearest to the nominal (bounded to the cell), so a safe nominal is left
unchanged and the target never jumps from one pixel center to another.

## 8. Realtime and threads

| Thread | Content | Rules |
|---|---|---|
| **realtime loop** (`controller_manager`, 1 kHz) | `update()`: estimation, MPC, torques | no dynamic allocation on the hot path, no blocking lock; reads buffers with `readFromRT()` |
| **ROS executor** (callbacks) | IMU, localization, `cmd_vel`, map, contacts | writes `RealtimeBuffer`s (`writeFromNonRT`); the map is converted to a `GridMap` **in the callback** |
| **`PerceptivePlanner`** (dedicated thread) | foothold safety + convex regions | woken by a `condition_variable` (or every 50 ms) |

Exchanges with the planner:
- **requests**: posted by the RT loop with `std::try_to_lock` (`postFootholdRequest`); if the lock
  is taken it retries on the next tick; fixed-size types;
- **results**: `RealtimeBuffer<FootholdResults>` (all 4 legs), read once per tick;
- **map**: immutable `std::shared_ptr<const GridMap>`, in a `RealtimeBuffer` for the RT loop and
  under a mutex for the thread; old maps are released on the writer side (outside the RT loop).

Each result carries the swing identifier (`swingId`): a result from an earlier swing is never
applied to the current one.

## 9. Files and functions

### `include/quadru_control/cheetah_controller.hpp`, `src/cheetah_controller.cpp`

The plugin: robot adaptation and state machine; the locomotion algorithm lives in
`cheetah_locomotion`.

| Function | Role |
|---|---|
| `command_interface_configuration()` | claims the 12 `effort` interfaces |
| `state_interface_configuration()` | claims 12 `position` then 12 `velocity` |
| `on_configure()` | creates `StandController`, publishers, parameters, `PerceptivePlanner`, `MitFootholdAdapter`, `FloatingBaseModel`, subscriptions (IMU, localization, map, `cmd_vel`, contacts) |
| `on_activate()` | resets to Stabilize, clears buffers and the adapter, (re)starts the planning thread |
| `on_deactivate()` | stops the planning thread |
| `update()` | control loop (§4) |
| `buildRobotParams(q0, R0)` | rigid-body parameters at the start pose (§6.5) |
| `readLegs(q, qd, legs)` | FK + Jacobian of each leg, in the MPC body frame, relative to the hip |
| `readImu(imu, q, gyro, acc)` | IMU (trunk frame) → MPC body frame |
| `legTorques(legs, cmds, q, qd)` | leg torques: stance law / OSC or spring swing (§6.12) |
| `trunkState()` | estimator state → trunk link state in the world |
| `command()` | velocity command from `cmd_vel` (or `walk_speed`) |
| `applyLocalization()` | offset position correction (§6.7) |
| `updatePerception(q)` | map → foothold adapter, ground height, pitch (§6.14) |
| `writeTorques(tau)` | writes the 12 torques |
| `publish(time, tau)` | `~/diag_feet`, `~/odom`, diagnostics |

### `include/quadru_control/controller_config.hpp`

Shared constants (single source): `kG`, `kTrunkCom`, `kFootRadius`, `kTauMax`, `kTauLimit`,
reference gait (`kTCycle`, `kDuty`, `kStepH`), foothold adaptation
(`kFootHeightSearchRadiusCells`, `kFootholdCommitPhase`, `kFootholdWindowHalf`), standing
(`kStabilizeDuration`, `kStandKp`, `kStandKd`), contacts (`kHysteresisConfirm`,
`kSensorTimeoutNs`).

### `include/quadru_control/robot_model.hpp`, `src/robot_model.cpp` — `RobotModel`

Robot mass properties (derived once from the URDF, no URDF parsing at runtime).

| Function | Role |
|---|---|
| `RobotModel()` | whole-body model at the nominal pose (14.58 kg, fixed inertia) — root project prototypes |
| `trunkOnly()` | trunk link only (5.00 kg, its inertia, MPC frame) + total mass |
| `trunkCom()` | trunk link CoM in its own frame |
| `compositeInertia(...)` | CoM and composite inertia at the current configuration (§6.3), in the MPC frame |
| `mass()`, `totalMass()`, `gravityAccel()`, `inertia()`, `nominalFeet()` | accessors |

### `include/quadru_control/leg_kinematics.hpp`, `src/leg_kinematics.cpp` — `LegKinematics`

Kinematics and mass properties of one leg from the exact URDF origins (validated against PyBullet:
2·10⁻⁸ m on the foot position).

| Function | Role |
|---|---|
| `forwardKinematics(q, &J)` | foot position (sphere center) and Jacobian, trunk frame (§6.1) |
| `chainPoints(q)` | hip, flex axis, knee, foot: leg collision check (criterion d) |
| `inverseKinematics(target, qInit, qOut, tol)` | damped least-squares IK (§6.1), criterion a |
| `gravityCompensation(q, g_body)` | leg gravity torque (§6.2) |
| `linkPoses(q)` | mass, CoM, inertia of the 3 links in the trunk frame (composite inertia) |
| `jointFrames()`, `linkInertials()`, `footOffset()` | raw data for `FloatingBaseModel` |
| `buildAllLegs()` | the 4 legs of the robot (URDF origins and inertias) |

### `include/quadru_control/floating_base_model.hpp`, `src/floating_base_model.cpp`, `include/quadru_control/spatial.hpp`

`FloatingBaseModel`: full floating-base dynamics (§6.4) — `massMatrix` (CRBA),
`inverseDynamics` (RNEA), `biasForces`, `contactJacobian`, `footBiasAcceleration`.
`spatial.hpp`: Featherstone spatial algebra — `skew`, `xlateRotate` (motion transform),
`forceTransform`, `crossMotion`, `crossForce`, `spatialInertia`.

### `include/quadru_control/stand_controller.hpp`, `src/stand_controller.cpp` — `StandController`

Standing controller (§6.13): `setTarget(q)`, `torques(q, qd, contacts, R, assumeAllFeet)`.

### `include/quadru_control/mit_foothold_adapter.hpp`, `src/mit_foothold_adapter.cpp` — `MitFootholdAdapter`

Implements `cheetah::FootholdAdapter`, the perception hook of the locomotion library.

| Element | Role |
|---|---|
| `FootholdMode` | `kNominal`, `kHeuristic`, `kConvexMpc` |
| `Context` | map of the tick, planner results, mode, trunk pose and velocity, joint angles |
| `setContext(ctx)` | set before every `locomotion_->run()` |
| `adapt(leg, swingPhase, swingTime, pFoot, Pf)` | per-swing state machine (§7.1); modifies $P_f$ |
| `footGround()` | ground height under each foot (estimator, body reference) |
| `commits` | footholds frozen this tick (published then cleared by the controller) |
| `reset()` | re-activation |

### `include/quadru_control/perceptive_planner.hpp`, `src/perceptive_planner.cpp` — `PerceptivePlanner`

Non-realtime planning thread (§8).

| Function | Role |
|---|---|
| `start(logger)`, `stop()`, `reset()` | thread lifecycle |
| `setTerrainMap(map)` | (ROS callback) new map |
| `readTerrainMap()` | (RT) map of the tick or `nullptr` |
| `postFootholdRequest(leg, req)` | (RT, non-blocking) evaluation request for one swing |
| `readFootholdResults()` | (RT) latest results of the 4 legs |
| `processPending()` | processes pending requests (thread, or tests) |
| `evaluateFoothold(leg, req)` | heightmap → safety → convex regions → result, evaluation time |

### `include/quadru_control/foothold_safety.hpp`, `src/foothold_safety.cpp`

Foothold safety and convex decomposition (§7.2–7.5): `extractFootholdHeightmap`,
`evaluateFootholdSafety` (+ `SafetyParams`, `CriteriaMask`, `SafetyStats`),
`decomposeConvexRegions`, `selectNearestRegion`, `nearestSafeFoothold`. Internal helpers:
`terrainAt`, `terrainUnderFoot`, `sphereHitsTerrain`, `limbsClearTerrain`, `swingPosition`
(same `FootSwingTrajectory` as the controller).

### `include/quadru_control/terrain_awareness.hpp`, `src/terrain_awareness.cpp`

`lookupTerrainHeight(map, x, y, radius)`: height (`elevation` layer) of the nearest **observed**
cell (finite variance) in a square neighborhood; `nullopt` otherwise (the map is sparse).

### `include/quadru_control/controller_publishers.hpp`, `src/controller_publishers.cpp` — `ControllerPublishers`

Realtime publishers (§2.3): `configure`, `publishOdom` (CoM → trunk link origin, velocities in
the trunk frame), `publishDiagnostics`, `publishFoothold`, `publishFootholdRegion`.
Structures `BodyState` and `ControllerDiagnostics`.

### `include/quadru_control/contact_sensing.hpp`, `src/contact_sensing.cpp`

Gazebo contacts (**diagnostics**): `contactSampleFromGazebo` (normal oriented towards the foot),
`ContactDebouncer` (leaky debouncer: +1 per new contact message, −1 otherwise, confirmed at 5,
each message counted once), `ContactSensing` (subscriptions, `read()` once per tick,
`contacts()`, `rawContacts()`).

### Other files

| File | Role |
|---|---|
| `quadru_control_plugin.xml` | pluginlib declaration of `quadru_control/CheetahController` |
| `CMakeLists.txt` | shared library `quadru_control` (all sources), depends on `terrain_mapping::terrain_processing` and `cheetah_locomotion::cheetah_locomotion`; gtest tests |
| `scripts/attitude_logger.py` | node logging `~/odom` to CSV (t, x, y, z, roll, pitch, yaw) — Fig. 3 of the paper |
| `scripts/plot_gazebo_run.py` | figures from a `ros2 bag` of the `diag_*` and `odom` topics. Written for the previous controller: some labels (ESKF, OSQP iterations, estimated force) no longer match the current topic content (§2.3) |

## 10. Tests

```bash
colcon test --base-paths src --packages-select quadru_control && colcon test-result --verbose
```

| Test | Checks |
|---|---|
| `test_configure_no_gazebo` | `on_configure()` returns within 5 s without Gazebo or hardware (otherwise the spawner hangs on "Configuring controller") |
| `test_update_warmup_dt_guard` | an abnormal period (1 s ≫ 50 ms) does not count towards the 0.1 s IMU wait: `~/diag_watchdog` only appears after ~100 normal ticks |
| `test_controller_modules` | debouncer (each message counted once); `StandController`: stance legs carry exactly $Mg$, the others only their own gravity |
| `test_foothold_safety` | roughness limited to the step edge; reachability; frontal collision along the actual trajectory; no false leg collision on flat ground; convex regions contain only safe cells and the nearest one is on the step; heuristic; foot resting against a riser ≠ collision |
| `test_perceptive_planner` | posting a request never blocks during an evaluation; results carry the swing id and the region |

## 11. Build, run, debug

```bash
cd quadru_ws && source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to quadru_control
source install/setup.bash

# Course with perception (stairs profile, config/mevius_controllers.yaml)
ros2 launch spawn_chien spawn.launch.py terrain_aware:=true \
  world:=$PWD/install/spawn_chien/share/spawn_chien/config/parcours_10cm.world

# Fast trot on flat ground (config/trot_flat.yaml, 0.8 m/s by default)
ros2 launch spawn_chien trot_flat.launch.py speed:=0.8
ros2 param set /mevius_mpc_controller walk_speed 1.0      # change the speed while walking
```

**Profiles** (same controller, different parameter files):

| | Stairs (`mevius_controllers.yaml`) | Flat trot (`trot_flat.yaml`) |
|---|---|---|
| gait | 0.65 s cycle, 80 % stance (144 ms swing) | 0.3 s cycle, 50 % stance (150 ms swing) |
| swing height | 8 cm | 6 cm |
| speed | 0.3 m/s | 0.8 m/s (`speed:=`) |
| terrain perception | yes (`terrain_aware:=true`) | no |
| command ramp / speed integral | off | 1.0 m/s² / 0.5 1/s |
| friction feedforward, rolling foot | off | on |

Measured in Gazebo (35 N·m): flat trot 0.806 / 0.807 m/s for 0.8 (2 runs), 0.503 for 0.5, 1.008
for 1.0, roll ≤ 4.9°, lateral drift ≤ 2 cm; stairs course complete 2/2 (pitch ≤ 25.5°).

```bash
ros2 topic echo /mevius_mpc_controller/diag_watchdog        # 3rd value = 1.0: Locomotion
ros2 topic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
ros2 param set /mevius_mpc_controller foothold_adaptation heuristic
ros2 param set /mevius_mpc_controller swing_control mit
ros2 run quadru_control attitude_logger.py --ros-args -p output:=/tmp/att.csv -p use_sim_time:=true
```

**Real-time control dashboard** (`scripts/control_dashboard.py`, pyqtgraph):

```bash
ros2 run quadru_control control_dashboard.py          # with any running simulation or robot
ros2 launch spawn_chien trot_flat.launch.py dashboard:=true
```

| Tab | Setpoint | Measurement |
|---|---|---|
| Joint torque | commanded τ (`~/diag_control`), ±τ_max | `/joint_states` effort (Gazebo echoes the command: ideal actuator; on the robot, the drive's measured torque) |
| Swing tracking | foot setpoint (hip-relative, x/y/z) | foot from joint encoders; swing error ‖p_des − p‖ |
| Ground forces | MPC force ‖F‖ and components | Gazebo bumper ‖F‖ (mean over physics steps); planned vs measured contact |
| Body | command, MPC reference | estimate, Gazebo ground truth moved to the trunk CoM |
| MPC | MPC period | QP solve time, QP status |

All curves use message timestamps (simulated time in Gazebo); *Pause* freezes the display.

In RViz: `/mevius_mpc_controller/foothold` (Point), `/mevius_mpc_controller/foothold_region`
(Polygon), `/terrain_mapping_node/elevation_cloud` (map).

## 12. Known limitations

- **Localization**: Gazebo ground truth in simulation; the real robot needs a visual(-inertial)
  odometry publishing the trunk pose on `localization_topic`.
- **Contacts**: planned phase only (no early/late touchdown detection); Gazebo bumpers are only
  used for diagnostics.
- **Footholds**: not optimized by the MPC (§7.4); one foothold frozen per swing, at mid-swing.
- **MPC model**: composite inertia frozen at the start pose; world inertia rotated by yaw only
  (valid for small roll/pitch — up to ~25° pitch measured on 10 cm stairs, safety limit 28.6°).
- **OSC swing control**: fixed-base leg inertia (diagonal block of $M$), no Coriolis terms.
