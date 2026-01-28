cmake_minimum_required(VERSION 3.8)
project(pecka_tvmc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pecka_tvmc_msg REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
)

set(COMMON_SOURCES
  src/PID-Controller/PID_controller.cpp
  src/thruster-config/thruster_config.cpp
)

add_executable(tvmc
  src/motion-controller/controller.cpp
  src/reporters/thrust.cpp
  ${COMMON_SOURCES}
)

ament_target_dependencies(tvmc
  rclcpp
  std_msgs
  ament_index_cpp
  pecka_tvmc_msg
)

add_executable(pwmc
  src/reporters/pwm.cpp
  ${COMMON_SOURCES}
)

ament_target_dependencies(pwmc
  rclcpp
  std_msgs
  ament_index_cpp
)

target_link_libraries(pwmc Eigen3::Eigen)

install(TARGETS tvmc pwmc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
