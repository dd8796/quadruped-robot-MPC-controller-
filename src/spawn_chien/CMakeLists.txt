cmake_minimum_required(VERSION 3.5)
project(spawn_chien)

# Accept CATKIN_INSTALL_INTO_PREFIX_ROOT if passed (compatibility with catkin/colcon)
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT "${CATKIN_INSTALL_INTO_PREFIX_ROOT}" CACHE INTERNAL "compatibility variable to silence unused-variable warnings")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_library(spawn_chien_home_pose SHARED src/home_pose_plugin.cpp)
target_include_directories(spawn_chien_home_pose PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(spawn_chien_home_pose ${GAZEBO_LIBRARIES})
ament_target_dependencies(spawn_chien_home_pose gazebo_ros std_msgs)
install(TARGETS spawn_chien_home_pose LIBRARY DESTINATION lib)

install(PROGRAMS
  scripts/spawn_robot.py
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

# Install assets (meshes) so they are available at runtime via package:// URIs
install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
