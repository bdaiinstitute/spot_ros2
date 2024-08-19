# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from spot_driver.launch.spot_launch_helpers import get_login_parameters, spot_has_arm


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    hardware_interface = LaunchConfiguration("hardware_interface").perform(context)
    has_arm = IfCondition(LaunchConfiguration("has_arm")).evaluate(context)

    # This will override the `has_arm` argument with the actual value from the robot.
    # The `has_arm` argument is still useful for testing different robot types in mock mode.
    login_info_string = ""
    if hardware_interface == "spot-sdk":
        config_file = LaunchConfiguration("config_file").perform(context)
        has_arm = spot_has_arm(config_file_path=config_file, spot_name="Test")
        username, password, hostname, _, _ = get_login_parameters(config_file)
        login_info_string = f" hostname:={hostname} username:={username} password:={password}"

    # Generate the robot description
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("spot_ros2_control"), "xacro", LaunchConfiguration("description_file")]
            ),
            " has_arm:=",
            str(has_arm),
            " hardware_interface_type:=",
            LaunchConfiguration("hardware_interface"),
            login_info_string,
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    # Configuration files
    controller_config_file = PathJoinSubstitution(
        [FindPackageShare("spot_ros2_control"), "config", LaunchConfiguration("controllers_config")]
    )

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
            arguments=[
                "-d",
                PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "rviz", "spot_ros2_control.rviz"]),
            ],
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
