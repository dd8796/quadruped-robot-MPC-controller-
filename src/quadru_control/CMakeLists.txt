cmake_minimum_required(VERSION 3.16)
project(quadru_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(terrain_mapping REQUIRED)
find_package(cheetah_locomotion REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)


# ---- controller plugin -----------------------------------------------------------------
# Robot description sources are also built by the root quadru_cpp project.
add_library(quadru_control SHARED
  src/robot_model.cpp
  src/leg_kinematics.cpp
  src/floating_base_model.cpp
  src/cheetah_controller.cpp
  src/mit_foothold_adapter.cpp
  src/contact_sensing.cpp
  src/terrain_awareness.cpp
  src/stand_controller.cpp
  src/controller_publishers.cpp
  src/foothold_safety.cpp
  src/perceptive_planner.cpp
)
target_include_directories(quadru_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(quadru_control PUBLIC Eigen3::Eigen PRIVATE Threads::Threads)
target_link_libraries(quadru_control PUBLIC terrain_mapping::terrain_processing)
target_link_libraries(quadru_control PUBLIC cheetah_locomotion::cheetah_locomotion)
ament_target_dependencies(quadru_control PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  gazebo_msgs
  std_msgs
  geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
)
pluginlib_export_plugin_description_file(controller_interface quadru_control_plugin.xml)

# Offline analysis scripts.
install(PROGRAMS
  scripts/plot_gazebo_run.py
  scripts/attitude_logger.py
  scripts/control_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS quadru_control
  EXPORT export_quadru_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_quadru_control HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib realtime_tools
  rclcpp rclcpp_lifecycle sensor_msgs nav_msgs gazebo_msgs std_msgs geometry_msgs
  grid_map_core grid_map_ros grid_map_msgs terrain_mapping
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_configure_no_gazebo test/test_configure_no_gazebo.cpp)
  target_link_libraries(test_configure_no_gazebo quadru_control)
  ament_target_dependencies(test_configure_no_gazebo rclcpp controller_interface)

  # Foothold safety and convex decomposition (arXiv 2307.14775, II-A/II-C).
  ament_add_gtest(test_foothold_safety test/test_foothold_safety.cpp)
  target_link_libraries(test_foothold_safety quadru_control)

  ament_add_gtest(test_perceptive_planner test/test_perceptive_planner.cpp)
  target_link_libraries(test_perceptive_planner quadru_control)

  ament_add_gtest(test_controller_modules test/test_controller_modules.cpp)
  target_link_libraries(test_controller_modules quadru_control)

  ament_add_gtest(test_update_warmup_dt_guard test/test_update_warmup_dt_guard.cpp)
  target_link_libraries(test_update_warmup_dt_guard quadru_control)
  ament_target_dependencies(test_update_warmup_dt_guard
    rclcpp controller_interface hardware_interface sensor_msgs std_msgs)
endif()

ament_package()
