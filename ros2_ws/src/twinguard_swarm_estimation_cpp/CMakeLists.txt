cmake_minimum_required(VERSION 3.16)
project(twinguard_swarm_estimation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(twinguard_swarm_integrity_cpp REQUIRED)

include_directories(include)

add_library(twinguard_estimation_core
  src/ekf_estimator.cpp
  src/visual_odometry.cpp
)

target_include_directories(twinguard_estimation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(twinguard_estimation_core
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

add_executable(visual_odometry_node
  src/visual_odometry_node.cpp
)

target_link_libraries(visual_odometry_node
  twinguard_estimation_core
)

ament_target_dependencies(visual_odometry_node
  rclcpp
  geometry_msgs
  diagnostic_msgs
  sensor_msgs
  cv_bridge
)

add_executable(ekf_integrity_node
  src/ekf_integrity_node.cpp
)

target_link_libraries(ekf_integrity_node
  twinguard_estimation_core
)

ament_target_dependencies(ekf_integrity_node
  rclcpp
  geometry_msgs
  diagnostic_msgs
  px4_msgs
  # Reuses TrustScorer from twinguard_integrity_core through the exported
  # twinguard_swarm_integrity_cpp ament package.
  twinguard_swarm_integrity_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS twinguard_estimation_core visual_odometry_node ekf_integrity_node
  EXPORT export_twinguard_estimation_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(twinguard_estimation_core)
ament_export_targets(export_twinguard_estimation_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV)

ament_package()
