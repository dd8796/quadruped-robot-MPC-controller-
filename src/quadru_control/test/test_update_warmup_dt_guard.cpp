// kMaxDt guard: an oversized period during warm-up must not count towards kWarmupDuration.
// Nothing is published on ~/diag_watchdog before initialization, which the test observes.
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "quadru_control/cheetah_controller.hpp"

namespace
{
constexpr int kNumJoints = quadru_control::CheetahController::kNumJoints;

// "leg1_abduction/effort" -> ("leg1_abduction", "effort")
std::pair<std::string, std::string> splitInterface(const std::string & full)
{
  const auto slash = full.find('/');
  return {full.substr(0, slash), full.substr(slash + 1)};
}
}  // namespace

TEST(UpdateWarmupDtGuard, OversizedFirstPeriodDoesNotSkipWarmup)
{
  rclcpp::init(0, nullptr);
  {
    auto controller = std::make_shared<quadru_control::CheetahController>();
    ASSERT_EQ(controller->init("mevius_mpc_controller_test"), controller_interface::return_type::OK);
    ASSERT_EQ(controller->configure().label(), "inactive");

    // Dummy interfaces: standing pose, zero velocity.
    std::array<double, kNumJoints> pos{}, vel{}, eff{};
    for (int leg = 0; leg < 4; ++leg) {
      pos[3 * leg] = 0.0;
      pos[3 * leg + 1] = 0.52611;
      pos[3 * leg + 2] = 1.11101567;
    }
    vel.fill(0.0);
    eff.fill(0.0);
    const auto stateNames = controller->state_interface_configuration().names;
    const auto commandNames = controller->command_interface_configuration().names;
    ASSERT_EQ(stateNames.size(), static_cast<std::size_t>(2 * kNumJoints));
    ASSERT_EQ(commandNames.size(), static_cast<std::size_t>(kNumJoints));

    std::vector<hardware_interface::StateInterface> stateStorage;
    std::vector<hardware_interface::CommandInterface> commandStorage;
    stateStorage.reserve(2 * kNumJoints);
    commandStorage.reserve(kNumJoints);
    for (int i = 0; i < 2 * kNumJoints; ++i) {
      const auto [joint, kind] = splitInterface(stateNames[i]);
      stateStorage.emplace_back(joint, kind, i < kNumJoints ? &pos[i] : &vel[i - kNumJoints]);
    }
    for (int i = 0; i < kNumJoints; ++i) {
      const auto [joint, kind] = splitInterface(commandNames[i]);
      commandStorage.emplace_back(joint, kind, &eff[i]);
    }

    std::vector<hardware_interface::LoanedStateInterface> loanedState;
    std::vector<hardware_interface::LoanedCommandInterface> loanedCommand;
    for (auto & s : stateStorage) {
      loanedState.emplace_back(s);
    }
    for (auto & c : commandStorage) {
      loanedCommand.emplace_back(c);
    }
    controller->assign_interfaces(std::move(loanedCommand), std::move(loanedState));

    ASSERT_EQ(controller->get_node()->activate().label(), "active");

    auto imuPub = controller->get_node()->create_publisher<sensor_msgs::msg::Imu>(
      "/chien/imu", rclcpp::SensorDataQoS());
    // A LifecyclePublisher created after activation must be activated explicitly.
    imuPub->on_activate();

    bool diagSeen = false;
    auto diagSub = controller->get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mevius_mpc_controller_test/diag_watchdog", rclcpp::SystemDefaultsQoS(),
      [&diagSeen](std_msgs::msg::Float64MultiArray::SharedPtr) { diagSeen = true; });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller->get_node()->get_node_base_interface());

    // Republish the IMU at the current sim time so it is never stale.
    auto publishImuAt = [&](const rclcpp::Time & stamp) {
      sensor_msgs::msg::Imu msg;
      msg.orientation.w = 1.0;
      msg.header.stamp = stamp;
      imuPub->publish(msg);
      executor.spin_some();
    };

    rclcpp::Time t(1, 0, RCL_ROS_TIME);  // start at 1 s

    // Tick 1: 1 s period (> kMaxDt), must not advance the warm-up.
    t = t + rclcpp::Duration::from_seconds(1.0);
    publishImuAt(t);
    controller->update(t, rclcpp::Duration::from_seconds(1.0));
    executor.spin_some();
    EXPECT_FALSE(diagSeen)
      << "diag_watchdog publie des le premier tick : le tick enorme a ete compte dans le "
         "warm-up au lieu d'etre rejete (regression du garde-fou dt).";

    // Normal 1 kHz ticks: no diagnostics before ~90 ticks (warm-up is 0.1 s).
    constexpr double kDt = 0.001;
    for (int i = 0; i < 90 && !diagSeen; ++i) {
      t = t + rclcpp::Duration::from_seconds(kDt);
      publishImuAt(t);
      controller->update(t, rclcpp::Duration::from_seconds(kDt));
      executor.spin_some();
    }
    EXPECT_FALSE(diagSeen)
      << "diag_watchdog publie avant d'avoir accumule kWarmupDuration en ticks normaux : le "
         "tick enorme du debut a ete partiellement compte.";

    // Then the warm-up completes.
    for (int i = 0; i < 30 && !diagSeen; ++i) {
      t = t + rclcpp::Duration::from_seconds(kDt);
      publishImuAt(t);
      controller->update(t, rclcpp::Duration::from_seconds(kDt));
      executor.spin_some();
    }
    EXPECT_TRUE(diagSeen) << "le warm-up ne se termine jamais avec une cadence normale.";

    controller->get_node()->deactivate();
    controller->release_interfaces();
  }
  rclcpp::shutdown();
}
