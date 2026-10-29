#pragma once
// Robot and controller constants shared by all quadru_control modules.
#include <cstdint>

#include <Eigen/Dense>

namespace quadru_control
{
namespace cfg
{

inline constexpr double kG = 9.81;
// Trunk link CoM in its own frame (URDF <inertial><origin>).
inline const Eigen::Vector3d kTrunkCom(-0.0520205, 0.00128595, 0.509659);
inline constexpr double kFootRadius = 0.02;   // m, foot contact sphere
inline constexpr double kTauMax = 35.0;       // N.m, LA8308 KV90 + 9:1 at ~42 A
inline constexpr double kTauLimit = 35.0;     // N.m, commanded torque saturation

// Reference gait (validated on 5 / 7.5 / 10 cm stairs): 144 ms swing, 8 cm swing height.
// Overridden by the gait_cycle_s / gait_duty parameters.
inline constexpr double kTCycle = 0.65;
inline constexpr double kDuty = 0.8;
inline constexpr double kStepH = 0.08;

// Foothold adaptation (arXiv 2307.14775). Ground height under the nominal foothold: nearest
// observed cell within 3 cells; a larger radius sees the step too early.
inline constexpr int kFootHeightSearchRadiusCells = 3;
inline constexpr double kFootholdCommitPhase = 0.5;     // foothold frozen after mid-swing
inline constexpr double kFootholdWindowHalf = 0.12;     // m, half-size of the foothold heightmap

// Stand before walking (unitree_guide FixedStand). Same gains as the gazebo_ros2_control
// position PID holding the robot at spawn, so the handover is smooth.
inline constexpr double kStabilizeDuration = 2.5;
inline constexpr double kStandKp = 300.0;
inline constexpr double kStandKd = 15.0;

// Gazebo contact debouncing; also the staleness timeout for cmd_vel.
inline constexpr int kHysteresisConfirm = 5;
inline constexpr std::int64_t kSensorTimeoutNs = 50000000;

}  // namespace cfg
}  // namespace quadru_control
