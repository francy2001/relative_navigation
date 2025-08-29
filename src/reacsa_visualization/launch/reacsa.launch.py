"""Visualize Reacsa and applied wrenches in RVIZ"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from orl_common.constants import *


def generate_launch_description():
    pkg_reacsa_visualization = get_package_share_directory("reacsa_visualization")
    rviz_config_path = os.path.join(pkg_reacsa_visualization, "rviz", "reacsa_config.rviz")

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time", default_value="True", description="Use simulation (Gazebo) clock if true. Default is true."
    )

    # Configure info text publisher
    visualization_publisher_node = Node(
        package="reacsa_visualization",
        name="visualization_publisher",
        executable="visualization_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        namespace="reacsa",
    )

    # Rviz launch
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
        namespace="reacsa",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            visualization_publisher_node,
            rviz,
        ]
    )
