cmake_minimum_required(VERSION 3.8)
project(pecka_tvmc)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)        # only if you have Python nodes
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/MotionCommand.msg"
  "msg/PwmValues.msg"
  "msg/SensorData.msg"
  "msg/Controller.msg"
  "msg/Axes.msg"
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# If you have C++ nodes inside src/, add them here later like:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp ${PROJECT_NAME})
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

ament_package()

