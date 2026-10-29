#include "quadru_control/contact_sensing.hpp"

#include <algorithm>
#include <string>

#include "quadru_control/controller_config.hpp"

namespace quadru_control
{

ContactSample contactSampleFromGazebo(
  const gazebo_msgs::msg::ContactsState & msg, std::int64_t fallbackStampNs)
{
  ContactSample sample;
  sample.contact = !msg.states.empty();
  sample.stampNs = rclcpp::Time(msg.header.stamp).nanoseconds();
  if (sample.stampNs == 0) {
    sample.stampNs = fallbackStampNs;
  }
  if (!sample.contact) {return sample;}
  const auto & contact = msg.states.front();
  const std::size_t count = std::min(
    contact.contact_positions.size(), contact.contact_normals.size());
  if (count == 0) {return sample;}
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    position += Eigen::Vector3d(
      contact.contact_positions[i].x, contact.contact_positions[i].y, contact.contact_positions[i].z);
    normal += Eigen::Vector3d(
      contact.contact_normals[i].x, contact.contact_normals[i].y, contact.contact_normals[i].z);
  }
  position /= static_cast<double>(count);
  const double normalNorm = normal.norm();
  if (normalNorm > 0.5 && position.allFinite()) {
    normal /= normalNorm;
    // The Gazebo/ODE normal sign depends on the body order: orient it towards the foot.
    if (normal.z() < 0.0) {normal = -normal;}
    sample.positionWorld = position;
    sample.normalWorld = normal;
    sample.hasGeometry = true;
  }
  return sample;
}

void ContactDebouncer::reset()
{
  count_.fill(0);
  lastStampNs_.fill(-1);
}

bool ContactDebouncer::isFresh(const ContactSample & sample, std::int64_t nowNs)
{
  const std::int64_t ageNs = nowNs - sample.stampNs;
  return sample.contact && ageNs >= 0 && ageNs <= cfg::kSensorTimeoutNs;
}

bool ContactDebouncer::update(int leg, const ContactSample & sample, std::int64_t nowNs)
{
  if (sample.stampNs != lastStampNs_[leg]) {
    lastStampNs_[leg] = sample.stampNs;
    count_[leg] += isFresh(sample, nowNs) ? 1 : -1;
    count_[leg] = std::clamp(count_[leg], 0, cfg::kHysteresisConfirm);
  }
  return count_[leg] >= cfg::kHysteresisConfirm;
}

void ContactSensing::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  for (int i = 0; i < 4; ++i) {
    buffers_[i].writeFromNonRT(ContactSample{});
    const std::string topic = "/chien/leg" + std::to_string(i + 1) + "_foot_contact";
    subs_[i] = node->create_subscription<gazebo_msgs::msg::ContactsState>(
      topic, rclcpp::SensorDataQoS(),
      [this, i, node](const gazebo_msgs::msg::ContactsState::SharedPtr msg) {
        buffers_[i].writeFromNonRT(contactSampleFromGazebo(*msg, node->now().nanoseconds()));
      });
  }
}

void ContactSensing::reset()
{
  debouncer_.reset();
  samples_.fill(ContactSample{});
  debounced_.fill(false);
  raw_.fill(false);
}

void ContactSensing::read(std::int64_t nowNs)
{
  for (int i = 0; i < 4; ++i) {
    samples_[i] = *buffers_[i].readFromRT();
    raw_[i] = ContactDebouncer::isFresh(samples_[i], nowNs);
    debounced_[i] = debouncer_.update(i, samples_[i], nowNs);
  }
}

}  // namespace quadru_control
