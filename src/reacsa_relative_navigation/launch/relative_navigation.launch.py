"""Launch the lidar_pose_estimation node"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true. Default is true."
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Lidar Pose Estimation node
    lidar_node = Node(
        package="reacsa_relative_navigation",
        executable="lidar_pose_estimation",
        name="lidar_pose_estimation",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    return LaunchDescription([declare_use_sim_time_cmd, lidar_node])
