# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file").perform(context)
    print("CONFIG FILE")
    print(config_file)
    # has_arm = spot_has_arm(config_file_path=config_file, spot_name="Test")

    # Generate the robot description
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("spot_ros2_control"), "xacro", LaunchConfiguration("description_file")]
            ),
            " has_arm:=",
            LaunchConfiguration("has_arm"),
            " hardware_interface_type:=",
            LaunchConfiguration("hardware_interface"),
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    # Configuration files
    controller_config_file = PathJoinSubstitution(
        [FindPackageShare("spot_ros2_control"), "config", LaunchConfiguration("controllers_config")]
    )
    rviz_config_file = PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "rviz", "spot_ros2_control.rviz"])

    # Nodes
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description, controller_config_file],
        )
    )
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[LaunchConfiguration("robot_controller"), "-c", "/controller_manager"],
        )
    )
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )
    )
    return


def generate_launch_description():
    # Populate launch description with launch arguments
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "has_arm",
                default_value="false",
                choices=["true", "false"],
                description="Whether the robot has an arm",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Path to general configuration file.",
            ),
            DeclareLaunchArgument(
                "controllers_config",
                default_value="spot_controllers_without_arm.yaml",
                description="YAML file for configuring the controllers.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="spot.urdf.xacro",
                description="URDF/XACRO description file with the robot.",
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="forward_position_controller",
                # This must match the controllers_config file. Right now this only has one option.
                choices=["forward_position_controller"],
                description="Robot controller to start.",
            ),
            DeclareLaunchArgument(
                "hardware_interface",
                default_value="mock",
                # Must match the xacro file options for which plugin to load
                choices=["mock", "spot-sdk"],
                description="Hardware interface to load",
            ),
        ]
    )
    # Add nodes to launch description
    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))
    return ld
