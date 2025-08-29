cmake_minimum_required(VERSION 3.5)
project(reacsa_visualization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(orl_common REQUIRED)
find_package(orl_interfaces REQUIRED)
find_package(canopen_interfaces REQUIRED)

# Add include directories
include_directories(include/reacsa_visualization)

# Add visualization publisher executable
add_executable(visualization_publisher src/visualization_publisher.cpp)

ament_target_dependencies(
  visualization_publisher
  ament_index_cpp
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2_geometry_msgs
  orl_common
  orl_interfaces
  canopen_interfaces)

install(TARGETS visualization_publisher
        DESTINATION lib/${PROJECT_NAME})

# Add visualization package directories
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
