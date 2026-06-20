cmake_minimum_required(VERSION 3.16)
project(twinguard_swarm_planning_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(include)

add_library(twinguard_planning_core
  src/astar_planner.cpp
  src/bt_nodes.cpp
)

target_include_directories(twinguard_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(twinguard_planning_core
  behaviortree_cpp_v3
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(
  TARGETS twinguard_planning_core
  EXPORT export_twinguard_planning_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(twinguard_planning_core)
ament_export_targets(export_twinguard_planning_core HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3)

ament_package()
