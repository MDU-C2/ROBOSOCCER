#===============================================================================
# Setup

# CMake version
cmake_minimum_required(VERSION 3.20)
# C++ standard
# Requirement to use 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED_ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

project(robot_controller_interface)

#===============================================================================
# find_package, include etc. Lastly add_subdirectory.

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(GTest REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

#===============================================================================
# add_subdirectories must be last

add_subdirectory(src)
add_subdirectory(test)

#===============================================================================
