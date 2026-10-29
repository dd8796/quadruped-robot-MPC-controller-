cmake_minimum_required(VERSION 3.8)
project(cheetah_locomotion LANGUAGES CXX)

# MIT Cheetah-Software convex MPC (c71c5a1, MIT license), adapted to this robot. No ROS
# dependency; see NOTICE.md.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---- qpOASES 3.2 (from Cheetah-Software/third-party, LGPL 2.1), shared library.
file(GLOB QPOASES_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/src/*.cpp)
add_library(cheetah_qpOASES SHARED ${QPOASES_SOURCES})
target_include_directories(cheetah_qpOASES PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/include>
  $<INSTALL_INTERFACE:include/qpOASES_cheetah>)
target_compile_definitions(cheetah_qpOASES PUBLIC __NO_COPYRIGHT__ LINUX)
target_compile_options(cheetah_qpOASES PRIVATE -O3 -w -finline-functions)

# ---- MIT convex MPC + state estimator
add_library(cheetah_locomotion SHARED
  src/ConvexMPCLocomotion.cpp
  src/StateEstimator.cpp
  src/SolverMPC.cpp
  src/convexMPC_interface.cpp
  src/RobotState.cpp
  src/Gait.cpp
  src/FootSwingTrajectory.cpp
)
target_include_directories(cheetah_locomotion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/cheetah_locomotion>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/cheetah_locomotion>)
target_link_libraries(cheetah_locomotion PUBLIC Eigen3::Eigen cheetah_qpOASES pthread)
target_compile_options(cheetah_locomotion PRIVATE -O3)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/qpOASES/include/ DESTINATION include/qpOASES_cheetah)
install(TARGETS cheetah_qpOASES cheetah_locomotion EXPORT export_cheetah_locomotion
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_cheetah_locomotion HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cheetah_locomotion test/test_cheetah_locomotion.cpp)
  target_link_libraries(test_cheetah_locomotion cheetah_locomotion)
endif()

ament_package()
