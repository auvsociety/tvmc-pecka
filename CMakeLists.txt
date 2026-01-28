cmake_minimum_required(VERSION 3.8)
project(pecka_tvmc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pecka_tvmc_msg REQUIRED)

add_library(pecka_tvmc_lib
  src/PID-Controller/PID_controller.cpp
  src/reporters/thrust.cpp
  src/thruster-config/thruster_config.cpp
  src/motion-controller/controller.cpp
)

target_include_directories(pecka_tvmc_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

ament_target_dependencies(pecka_tvmc_lib
  rclcpp
  std_msgs
  ament_index_cpp
  pecka_tvmc_msg
)

install(TARGETS pecka_tvmc_lib
  DESTINATION lib
)

ament_package()
