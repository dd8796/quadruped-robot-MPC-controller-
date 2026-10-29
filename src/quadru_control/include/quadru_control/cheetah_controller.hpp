#pragma once
// ros2_control plugin running the MIT Cheetah-Software locomotion (cheetah_locomotion) on this
// robot: robot adaptation, state machine, terrain perception and external localization.
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "quadru_control/floating_base_model.hpp"
#include "quadru_control/leg_kinematics.hpp"
#include "quadru_control/robot_model.hpp"

#include "cheetah_locomotion/ConvexMPCLocomotion.h"
#include "cheetah_locomotion/StateEstimator.h"
#include "quadru_control/contact_sensing.hpp"
#include "quadru_control/controller_publishers.hpp"
#include "quadru_control/mit_foothold_adapter.hpp"
#include "quadru_control/perceptive_planner.hpp"
#include "quadru_control/stand_controller.hpp"

namespace quadru_control
{

class CheetahController : public controller_interface::ControllerInterface
{
public:
  CheetahController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  static constexpr int kNumJoints = 12;
  enum class ControlState { kStabilize = 0, kLocomotion = 1, kFault = 2 };

private:
  using Vector12 = Eigen::Matrix<double, 12, 1>;

  // MIT robot parameters at the start pose q0, R0 (trunk -> world).
  cheetah::RobotParams buildRobotParams(const Vector12 & q0, const Eigen::Matrix3d & R0);
  // Encoders -> MIT leg data (foot relative to hip, MIT body frame).
  void readLegs(const Vector12 & q, const Vector12 & qd, cheetah::LegData legs[4]) const;
  // Estimator input: in stance the foot sphere rolls, so the point at rest is its contact point,
  // not its center. Replaces each foot velocity with that of the contact point (foot_rolling_comp).
  void contactPointVelocities(
    const Vector12 & q, const Vector12 & qd, const Quat<float> & qMit, const Vec3<float> & gyroMit,
    cheetah::LegData legs[4]) const;
  // IMU (trunk frame) -> MIT body frame.
  void readImu(
    const sensor_msgs::msg::Imu & imu, Quat<float> & q, Vec3<float> & gyro, Vec3<float> & acc) const;
  // Stance: MIT LegController law. Swing: operational-space control or MIT law (swing_control).
  Vector12 legTorques(
    const cheetah::LegData legs[4], const cheetah::LegCommand cmds[4], const Vector12 & q,
    const Vector12 & qd) const;
  BodyState trunkState() const;   // MIT state -> trunk link state (world)
  // Velocity command (cmd_vel, else walk_speed), rate-limited by cmd_accel_limit.
  cheetah::LocomotionCommand command();
  cheetah::LocomotionCommand commandTarget();
  void updatePerception(const Vector12 & q);
  void writeTorques(const Vector12 & tau);
  void publish(const rclcpp::Time & time, const Vector12 & tau);
  void publishControl(
    const rclcpp::Time & time, const Vector12 & tau, const Vector12 & q, const Vector12 & qd,
    const cheetah::LegData legs[4]);

  std::array<LegKinematics, 4> legs_;
  RobotModel model_;
  std::unique_ptr<StandController> stand_;
  std::unique_ptr<cheetah::ConvexMPCLocomotion> locomotion_;
  std::unique_ptr<cheetah::StateEstimator> estimator_;
  cheetah::RobotParams robot_;
  ContactSensing contacts_;   // Gazebo bumpers, diagnostics only
  ControllerPublishers publishers_;
  // ~/diag_feet: [foot center setpoint (x, y, z, world) x4, swing flag x4]
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> feetPub_;
  // ~/diag_control (Locomotion, every kControlDiagDecimation ticks): setpoints next to their
  // measurements, layout in kControlDiagLayout (scripts/control_dashboard.py).
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> controlPub_;
  int controlDiagTick_ = 0;
  cheetah::LocomotionCommand sentCmd_;
  std::unique_ptr<PerceptivePlanner> planner_;
  std::unique_ptr<MitFootholdAdapter> footholds_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrainMapSub_;
  std::atomic<bool> terrainAware_{false};
  std::atomic<int> footholdMode_{static_cast<int>(FootholdMode::kConvexMpc)};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;
  struct LocalizationSample
  {
    Eigen::Vector3d trunkPosition = Eigen::Vector3d::Zero();   // trunk link origin, world
    Eigen::Quaterniond trunkOrientation = Eigen::Quaterniond::Identity();
    std::int64_t stampNs = 0;   // steady_clock
  };
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localizationSub_;
  realtime_tools::RealtimeBuffer<LocalizationSample> localizationBuffer_;
  std::atomic<bool> localizationReceived_{false};
  std::atomic<bool> useLocalization_{true};   // false: leg odometry only
  // Swing control: OSC F = Lambda (a + w^2 e + 2 zeta w de) + leg gravity (default), or the MIT
  // Cartesian spring, tuned for ~0.6 kg Mini Cheetah legs (reaches only 66-73 % of the apex here).
  std::atomic<bool> swingOsc_{true};
  std::atomic<double> swingOmega_{50.0};   // rad/s
  std::atomic<double> swingZeta_{1.0};
  // Joint friction feedforward tau += b qd + f tanh(qd / 0.05) (joint_damping_comp, joint_friction_comp).
  std::atomic<double> jointDampingComp_{0.0};   // N.m.s/rad
  std::atomic<double> jointFrictionComp_{0.0};  // N.m
  std::atomic<bool> footRollingComp_{false};
  std::unique_ptr<FloatingBaseModel> dynamics_;   // CRBA, leg inertia for OSC
  void applyLocalization();
  // REP-105 style correction: each new localization sample updates the offset
  // (localized - estimated position); the estimator itself stays continuous.
  Eigen::Vector3f locOffset_ = Eigen::Vector3f::Zero();
  bool haveLocOffset_ = false;
  std::int64_t lastLocStampNs_ = 0;

  // MIT body origin: whole-body CoM in the trunk frame, at the start pose.
  Eigen::Vector3d comTrunk_ = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 4> abdOriginTrunk_{};

  ControlState state_ = ControlState::kStabilize;
  bool firstUpdate_ = true;
  bool initialized_ = false;
  bool faultLogged_ = false;
  double warmupElapsed_ = 0.0;
  double t_ = 0.0;
  double dt_ = 0.001;
  Eigen::Matrix3d faultR_ = Eigen::Matrix3d::Identity();
  Vec4<float> contactPhase_ = Vec4<float>::Constant(0.5f);
  cheetah::LegCommand legCmds_[4];
  std::array<double, 4> mpcStats_{};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::Imu> imuBuffer_;
  std::atomic<bool> imuReceived_{false};
  struct CmdVelSample
  {
    double vx = 0.0, vy = 0.0, wz = 0.0;
    std::int64_t stampNs = 0;
  };
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  realtime_tools::RealtimeBuffer<CmdVelSample> cmdVelBuffer_;
  std::atomic<bool> cmdVelEverReceived_{false};
  std::atomic<double> walkSpeed_{0.3};       // m/s, used when no cmd_vel was ever received
  std::atomic<double> cmdAccelLimit_{0.0};   // m/s^2, 0 = no limit
  // Speed integral (vel_integral_gain, 1/s, 0 = off): removes the steady-state speed error of the
  // MPC, like the pitch/roll/height integrals of the locomotion. Bounded by kVelIntegralMax.
  std::atomic<double> velIntegralGain_{0.0};
  Eigen::Vector2d velIntegral_ = Eigen::Vector2d::Zero();
  cheetah::LocomotionCommand lastCmd_;
  double gaitCycle_ = 0.65;   // s
  double gaitDuty_ = 0.8;     // stance fraction
  double swingHeight_ = 0.08; // m
};

}  // namespace quadru_control
