#include "quadru_control/controller_publishers.hpp"

#include "quadru_control/controller_config.hpp"

namespace quadru_control
{

namespace
{
template<class T>
std::shared_ptr<realtime_tools::RealtimePublisher<T>> makePub(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & topic,
  const rclcpp::QoS & qos)
{
  return std::make_shared<realtime_tools::RealtimePublisher<T>>(
    node->create_publisher<T>(topic, qos));
}
}  // namespace

void ControllerPublishers::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  const auto qos = rclcpp::SystemDefaultsQoS();
  odom_ = makePub<nav_msgs::msg::Odometry>(node, "~/odom", qos);
  foothold_ = makePub<geometry_msgs::msg::PointStamped>(node, "~/foothold", rclcpp::QoS(10));
  footholdRegion_ = makePub<geometry_msgs::msg::PolygonStamped>(node, "~/foothold_region", rclcpp::QoS(10));
  contact_ = makePub<std_msgs::msg::Float64MultiArray>(node, "~/diag_contact", qos);
  force_ = makePub<std_msgs::msg::Float64MultiArray>(node, "~/diag_force", qos);
  torque_ = makePub<std_msgs::msg::Float64MultiArray>(node, "~/diag_torques", qos);
  watchdog_ = makePub<std_msgs::msg::Float64MultiArray>(node, "~/diag_watchdog", qos);
  mpc_ = makePub<std_msgs::msg::Float64MultiArray>(node, "~/diag_mpc", qos);
}

void ControllerPublishers::publishOdom(const rclcpp::Time & time, const BodyState & body)
{
  if (!odom_ || !odom_->trylock()) {return;}
  // The estimate is the CoM; odom -> trunk must carry the trunk link origin.
  const Eigen::Vector3d pTrunk = body.p - body.R * cfg::kTrunkCom;
  auto & msg = odom_->msg_;
  msg.header.stamp = time;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "trunk";
  msg.pose.pose.position.x = pTrunk.x();
  msg.pose.pose.position.y = pTrunk.y();
  msg.pose.pose.position.z = pTrunk.z();
  const Eigen::Quaterniond quat(body.R);
  msg.pose.pose.orientation.x = quat.x();
  msg.pose.pose.orientation.y = quat.y();
  msg.pose.pose.orientation.z = quat.z();
  msg.pose.pose.orientation.w = quat.w();
  const Eigen::Vector3d vBody = body.R.transpose() * body.v;
  const Eigen::Vector3d wBody = body.R.transpose() * body.w;
  msg.twist.twist.linear.x = vBody.x();
  msg.twist.twist.linear.y = vBody.y();
  msg.twist.twist.linear.z = vBody.z();
  msg.twist.twist.angular.x = wBody.x();
  msg.twist.twist.angular.y = wBody.y();
  msg.twist.twist.angular.z = wBody.z();
  odom_->unlockAndPublish();
}

void ControllerPublishers::publishDiagnostics(const ControllerDiagnostics & d)
{
  if (contact_ && contact_->trylock()) {
    auto & data = contact_->msg_.data;
    data.resize(12);
    for (int i = 0; i < 4; ++i) {
      data[i] = d.plannedContact[i] ? 1.0 : 0.0;
      data[4 + i] = d.estimatedContact[i] ? 1.0 : 0.0;
      data[8 + i] = d.rawContact[i] ? 1.0 : 0.0;
    }
    contact_->unlockAndPublish();
  }
  if (force_ && force_->trylock()) {
    auto & data = force_->msg_.data;
    data.resize(8);
    for (int i = 0; i < 4; ++i) {
      data[i] = d.commandedForceZ[i];
      data[4 + i] = d.estimatedForceZ[i];
    }
    force_->unlockAndPublish();
  }
  if (torque_ && torque_->trylock()) {
    torque_->msg_.data.assign(d.tau.data(), d.tau.data() + 12);
    torque_->unlockAndPublish();
  }
  if (watchdog_ && watchdog_->trylock()) {
    watchdog_->msg_.data = {d.tiltRad, d.heightErr, static_cast<double>(d.state)};
    watchdog_->unlockAndPublish();
  }
  if (mpc_ && mpc_->trylock()) {
    mpc_->msg_.data.assign(d.mpcStats.begin(), d.mpcStats.end());
    mpc_->unlockAndPublish();
  }
}

void ControllerPublishers::publishFoothold(const rclcpp::Time & time, const Eigen::Vector3d & p)
{
  if (!foothold_ || !foothold_->trylock()) {return;}
  foothold_->msg_.header.stamp = time;
  foothold_->msg_.header.frame_id = "odom";
  foothold_->msg_.point.x = p.x();
  foothold_->msg_.point.y = p.y();
  foothold_->msg_.point.z = p.z();
  foothold_->unlockAndPublish();
}

void ControllerPublishers::publishFootholdRegion(const rclcpp::Time & time, const ConvexRegion & r)
{
  if (!footholdRegion_ || !footholdRegion_->trylock()) {return;}
  auto & poly = footholdRegion_->msg_;
  poly.header.stamp = time;
  poly.header.frame_id = "odom";
  poly.polygon.points.resize(4);
  const double xs[4] = {r.min.x(), r.max.x(), r.max.x(), r.min.x()};
  const double ys[4] = {r.min.y(), r.min.y(), r.max.y(), r.max.y()};
  for (int c = 0; c < 4; ++c) {
    poly.polygon.points[c].x = static_cast<float>(xs[c]);
    poly.polygon.points[c].y = static_cast<float>(ys[c]);
    poly.polygon.points[c].z = static_cast<float>(r.height);
  }
  footholdRegion_->unlockAndPublish();
}

}  // namespace quadru_control
