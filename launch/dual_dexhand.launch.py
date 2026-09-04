#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    left_enabled = LaunchConfiguration("left")
    right_enabled = LaunchConfiguration("right")
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    node_id = LaunchConfiguration("node_id")
    home_on_start = LaunchConfiguration("home_on_start")
    velocity = LaunchConfiguration("velocity")

    common = {
        "node_id": ParameterValue(node_id, value_type=int),
        "velocity": ParameterValue(velocity, value_type=int),
        "auto_init": True,
        "home_on_init": ParameterValue(home_on_start, value_type=bool),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("left", default_value="true"),
            DeclareLaunchArgument("right", default_value="true"),
            DeclareLaunchArgument("left_interface", default_value="can0"),
            DeclareLaunchArgument("right_interface", default_value="can3"),
            DeclareLaunchArgument("node_id", default_value="1"),
            DeclareLaunchArgument("velocity", default_value="8000"),
            DeclareLaunchArgument(
                "home_on_start",
                default_value="true",
                description="Enable and home both RP_Hands during startup.",
            ),
            Node(
                package="roboparty_dexhand_ros",
                executable="dexhand_node",
                name="dexhand_left",
                parameters=[{**common, "interface": left_interface}],
                output="screen",
                condition=IfCondition(left_enabled),
            ),
            Node(
                package="roboparty_dexhand_ros",
                executable="dexhand_node",
                name="dexhand_right",
                parameters=[{**common, "interface": right_interface}],
                output="screen",
                condition=IfCondition(right_enabled),
            ),
        ]
    )
