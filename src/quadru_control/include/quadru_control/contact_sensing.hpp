#pragma once
// Foot contact from Gazebo bumpers (simulation and diagnostics only), with debouncing.
#include <array>
#include <cstdint>

#include <Eigen/Dense>

#include "gazebo_msgs/msg/contacts_state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace quadru_control
{

struct ContactSample
{
  bool contact = false;
  bool hasGeometry = false;      // contact position/normal valid
  std::int64_t stampNs = 0;
  Eigen::Vector3d positionWorld = Eigen::Vector3d::Zero();
  Eigen::Vector3d normalWorld = Eigen::Vector3d::UnitZ();   // points towards the foot
};

ContactSample contactSampleFromGazebo(
  const gazebo_msgs::msg::ContactsState & msg, std::int64_t fallbackStampNs);

// Leaky debouncer: +1 per new contact message, -1 otherwise, confirmed at kHysteresisConfirm.
// Each message is counted once (the controller runs faster than the sensor).
class ContactDebouncer
{
public:
  void reset();
  bool update(int leg, const ContactSample & sample, std::int64_t nowNs);
  // Fresh, unfiltered contact.
  static bool isFresh(const ContactSample & sample, std::int64_t nowNs);

private:
  std::array<int, 4> count_{};
  std::array<std::int64_t, 4> lastStampNs_{-1, -1, -1, -1};
};

// Subscribes to /chien/legN_foot_contact; call read() once per tick.
class ContactSensing
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void reset();
  void read(std::int64_t nowNs);
  const std::array<bool, 4> & contacts() const {return debounced_;}
  const std::array<bool, 4> & rawContacts() const {return raw_;}
  const std::array<ContactSample, 4> & samples() const {return samples_;}

private:
  std::array<rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr, 4> subs_;
  std::array<realtime_tools::RealtimeBuffer<ContactSample>, 4> buffers_;
  ContactDebouncer debouncer_;
  std::array<ContactSample, 4> samples_{};
  std::array<bool, 4> debounced_{};
  std::array<bool, 4> raw_{};
};

}  // namespace quadru_control
