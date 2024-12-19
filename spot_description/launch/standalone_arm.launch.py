# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

import os

import launch
import launch_ros
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description() -> launch.LaunchDescription:
    pkg_share = launch_ros.substitutions.FindPackageShare(package="spot_description").find("spot_description")
    default_model_path = os.path.join(pkg_share, "urdf/standalone_arm.urdf.xacro")
    default_rviz2_path = os.path.join(pkg_share, "rviz/standalone_arm.rviz")
    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="gui", default_value="True", description="Flag to enable joint_state_publisher_gui"
            ),
            launch.actions.DeclareLaunchArgument(
                name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig", default_value=default_rviz2_path, description="Absolute path to rviz config file"
            ),
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}],
            ),
            launch_ros.actions.Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
            ),
            launch_ros.actions.Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d" + default_rviz2_path],
            ),
        ]
    )
