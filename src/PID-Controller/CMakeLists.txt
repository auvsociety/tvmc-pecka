cmake_minimum_required(VERSION 3.13)  # CMake version check
project(PID_controller)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard
add_library(PID_controller PID_controller.cpp)
