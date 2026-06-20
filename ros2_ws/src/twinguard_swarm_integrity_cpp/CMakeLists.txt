cmake_minimum_required(VERSION 3.16)
project(twinguard_swarm_integrity_cpp)

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
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(twinguard_swarm_planning_cpp REQUIRED)

include_directories(include)

add_library(twinguard_integrity_core
  src/trust_scorer.cpp
  src/offboard_supervisor.cpp
)

target_include_directories(twinguard_integrity_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(integrity_node_cpp
  src/integrity_node.cpp
)

target_link_libraries(integrity_node_cpp
  twinguard_integrity_core
)

ament_target_dependencies(integrity_node_cpp
  rclcpp
  std_msgs
  geometry_msgs
  diagnostic_msgs
  px4_msgs
)

add_executable(formation_supervisor_node
  src/formation_supervisor_node.cpp
)

target_link_libraries(formation_supervisor_node
  twinguard_integrity_core
)

ament_target_dependencies(formation_supervisor_node
  rclcpp
  std_msgs
  geometry_msgs
  diagnostic_msgs
  px4_msgs
  ament_index_cpp
  behaviortree_cpp_v3
  twinguard_swarm_planning_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS twinguard_integrity_core integrity_node_cpp formation_supervisor_node
  EXPORT export_twinguard_integrity_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_twinguard_integrity_core HAS_LIBRARY_TARGET)

ament_package()
