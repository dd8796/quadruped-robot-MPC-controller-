cmake_minimum_required(VERSION 3.16)
project(terrain_mapping)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

# Map processing library, shared with quadru_control.
add_library(terrain_processing SHARED src/terrain_processing.cpp)
target_include_directories(terrain_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(terrain_processing PUBLIC grid_map_core)

add_executable(terrain_mapping_node src/terrain_mapping_node.cpp)
target_link_libraries(terrain_mapping_node terrain_processing)
target_include_directories(terrain_mapping_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(terrain_mapping_node ${PCL_LIBRARIES})
ament_target_dependencies(terrain_mapping_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

install(TARGETS terrain_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS terrain_processing
  EXPORT export_terrain_processing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_terrain_processing HAS_LIBRARY_TARGET)
ament_export_dependencies(grid_map_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_processing test/test_terrain_processing.cpp)
  target_link_libraries(test_terrain_processing terrain_processing)
endif()
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
