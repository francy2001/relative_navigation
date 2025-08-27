cmake_minimum_required(VERSION 3.5)
project(reacsa_relative_navigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add include directories
include_directories(include/reacsa_relative_navigation)

# Add executable
add_executable(lidar_pose_estimation src/lidar_pose_estimation.cpp)
ament_export_include_directories(include)

ament_target_dependencies(lidar_pose_estimation
    rclcpp
    sensor_msgs
    std_msgs
    PCL
    pcl_conversions
    tf2_ros
    tf2_sensor_msgs
    geometry_msgs
    visualization_msgs
)

install(TARGETS lidar_pose_estimation DESTINATION lib/${PROJECT_NAME})
target_link_libraries(lidar_pose_estimation ${PCL_LIBRARIES})
# Add launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
