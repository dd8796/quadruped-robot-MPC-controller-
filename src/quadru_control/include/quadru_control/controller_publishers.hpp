#pragma once
// Realtime publishers of the controller (non-blocking trylock). See README for the topic layout.
#include <array>
#include <memory>

#include <Eigen/Dense>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "quadru_control/foothold_safety.hpp"

namespace quadru_control
{

struct BodyState
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();   // trunk -> world
  Eigen::Vector3d p = Eigen::Vector3d::Zero();       // trunk link CoM, world
  Eigen::Vector3d v = Eigen::Vector3d::Zero();       // world
  Eigen::Vector3d w = Eigen::Vector3d::Zero();       // world
};

struct ControllerDiagnostics
{
  std::array<bool, 4> plannedContact{};
  std::array<bool, 4> estimatedContact{};
  std::array<bool, 4> rawContact{};
  std::array<double, 4> commandedForceZ{};   // URDF leg order
  std::array<double, 4> estimatedForceZ{};
  Eigen::Matrix<double, 12, 1> tau = Eigen::Matrix<double, 12, 1>::Zero();
  double tiltRad = 0.0;
  double heightErr = 0.0;
  int state = 0;
  std::array<double, 4> mpcStats{};
};

class ControllerPublishers
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void publishOdom(const rclcpp::Time & time, const BodyState & body);
  void publishDiagnostics(const ControllerDiagnostics & diag);
  void publishFoothold(const rclcpp::Time & time, const Eigen::Vector3d & pointWorld);
  void publishFootholdRegion(const rclcpp::Time & time, const ConvexRegion & region);

private:
  template<class T>
  using Pub = std::shared_ptr<realtime_tools::RealtimePublisher<T>>;
  Pub<nav_msgs::msg::Odometry> odom_;
  Pub<geometry_msgs::msg::PointStamped> foothold_;
  Pub<geometry_msgs::msg::PolygonStamped> footholdRegion_;
  Pub<std_msgs::msg::Float64MultiArray> contact_, force_, torque_, watchdog_, mpc_;
};

}  // namespace quadru_control
