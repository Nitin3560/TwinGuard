cmake_minimum_required(VERSION 3.8)
project(twinguard_swarm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_integrity_supervisor_pipeline.py TIMEOUT 60)
endif()

ament_package()
