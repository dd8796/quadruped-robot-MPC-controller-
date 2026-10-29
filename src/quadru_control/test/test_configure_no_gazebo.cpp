// on_configure() without Gazebo or hardware must return quickly, otherwise the
// gazebo_ros2_control spawner hangs on "Configuring controller".
#include <chrono>
#include <cstdio>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "quadru_control/cheetah_controller.hpp"

TEST(ConfigureNoGazebo, ConfigureReturnsWithinTimeout)
{
  rclcpp::init(0, nullptr);
  {
    auto controller = std::make_shared<quadru_control::CheetahController>();
    auto init_ret = controller->init("mevius_mpc_controller_test");
    ASSERT_EQ(init_ret, controller_interface::return_type::OK);
    std::fprintf(stderr, "[debug] init() done\n");

    auto t0 = std::chrono::steady_clock::now();
    const auto & state = controller->configure();
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::fprintf(stderr, "[debug] configure() done in %.1f ms, state=%s\n", ms, state.label().c_str());

    EXPECT_LT(ms, 5000.0);
    EXPECT_EQ(state.label(), "inactive");
  }
  rclcpp::shutdown();
}
