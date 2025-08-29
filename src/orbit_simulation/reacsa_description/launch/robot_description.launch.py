#!/usr/bin/env python3
"""Create a simulation of the ORL flatfloor using Gazebo and spawn the reacsa within it"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from orl_common.constants import *

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Specify paths, robot name to load and path to config file within the package
    pkg_reacsa_description = get_package_share_directory("reacsa_description")
    reacsa_models_path = os.path.join(pkg_reacsa_description, "urdf")
    reacsa_xacro_path = os.path.join(reacsa_models_path, "REACSA.urdf.xacro")

    robot_description = xacro.process_file(
        reacsa_xacro_path,
        mappings={
            "namespace": "/reacsa",
            "height_acrobat_bottom_plate": str(HEIGHT_ACROBAT_BOTTOM_PLATE),
            "height_acrobat_center": str(HEIGHT_ACROBAT_CENTER),
            "height_acrobat_top_plate": str(HEIGHT_ACROBAT_TOP_PLATE),
            "height_air_bearing_200mm": str(HEIGHT_AIR_BEARING_200MM),
            "height_acrobat_leg": str(HEIGHT_ACROBAT_LEG),
            "radius_acrobat_bottom_plate": str(RADIUS_ACROBAT_BOTTOM_PLATE),
            "radius_acrobat_center": str(RADIUS_ACROBAT_CENTER),
            "radius_acrobat_top_plate": str(RADIUS_ACROBAT_TOP_PLATE),
            "radius_air_bearing_200mm": str(RADIUS_AIR_BEARING_200MM),
            "radius_acrobat_leg": str(RADIUS_ACROBAT_LEG),
            "radius_acrobat_air_bearing_200mm": str(RADIUS_ACROBAT_AIR_BEARING),
            "mass_air_bearing_200mm": str(MASS_AIR_BEARING_200MM),
            "mass_acrobat_total": str(MASS_ACROBAT_TOTAL),
            "inertia_acrobat": str(INERTIA_ACROBAT),
            "height_recap_bottom_plate": str(HEIGHT_RECAP_BOTTOM_PLATE),
            "height_reaction_wheel": str(HEIGHT_REACTION_WHEEL),
            "height_recap_center": str(HEIGHT_RECAP_CENTER),
            "height_recap_top_plate": str(HEIGHT_RECAP_TOP_PLATE),
            "height_recap_column": str(HEIGHT_RECAP_COLUMN),
            "radius_reaction_wheel": str(RADIUS_REACTION_WHEEL),
            "radius_recap_center": str(RADIUS_RECAP_CENTER),
            "mass_recap_total": str(MASS_RECAP_TOTAL),
            "mass_reaction_wheel": str(MASS_REACTION_WHEEL),
            "inertia_reaction_wheel": str(INERTIA_REACTION_WHEEL),
            "inertia_recap": str(INERTIA_RECAP),
            "init_velocity_recap": str(VELOCITY_REACTION_WHEEL_INITIAL * RPM_2_RADPS),
            "height_satsim_bottom_plate": str(HEIGHT_SATSIM_BOTTOM_PLATE),
            "length_thruster": str(LENGTH_THRUSTER),
            "height_satsim_center": str(HEIGHT_SATSIM_CENTER),
            "height_satsim_top_plate": str(HEIGHT_SATSIM_TOP_PLATE),
            "radius_satsim_bottom_plate": str(RADIUS_SATSIM_BOTTOM_PLATE),
            "radius_thruster": str(RADIUS_THRUSTER),
            "mass_thruster": str(MASS_THRUSTER),
            "mass_satsim_total": str(MASS_SATSIM_TOTAL),
            "inertia_satsim": str(INERTIA_SATSIM),
            "mass_camera": str(MASS_CAMERA),
        },
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_description},
            {"publish_frequency": 10.0},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        namespace="reacsa",
    )

    # base_link to world TF broadcaster
    base_tf_broadcaster_node = Node(
        package="orl_common",
        name="base_tf_broadcaster",
        executable="base_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("/tf", "tf"),
        ],
        namespace="reacsa",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            base_tf_broadcaster_node,
        ]
    )
