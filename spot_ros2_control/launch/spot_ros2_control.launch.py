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
    hardware_interface: str = LaunchConfiguration("hardware_interface").perform(context)
    controllers_config: str = LaunchConfiguration("controllers_config").perform(context)
    mock_has_arm: bool = IfCondition(LaunchConfiguration("mock_has_arm")).evaluate(context)

    # If connected to a physical robot, query if it has an arm. Otherwise, use the value in mock_has_arm.
    login_info_string = ""
    if hardware_interface == "spot-sdk":
        config_file = LaunchConfiguration("config_file").perform(context)
        has_arm = spot_has_arm(config_file_path=config_file, spot_name="Test")
        username, password, hostname, _, _ = get_login_parameters(config_file)
        login_info_string = f" hostname:={hostname} username:={username} password:={password}"
    else:
        has_arm = mock_has_arm

    # Generate the robot description based off the arm status.
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "xacro", "spot.urdf.xacro"]),
            " has_arm:=",
            str(has_arm),
            " hardware_interface_type:=",
            LaunchConfiguration("hardware_interface"),
            login_info_string,
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    # Configuration for the controller.
    # If not controller is selected, use the appropriate default given if the robot has an arm or not.
    # Else, just use the yaml that is passed in.
    if controllers_config == "":
        default_config_file = "spot_controllers_with_arm.yaml" if has_arm else "spot_controllers_without_arm.yaml"
        controllers_config = PathJoinSubstitution(
            [FindPackageShare("spot_ros2_control"), "config", default_config_file]
        )

    # Add nodes
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description, controllers_config],
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
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )
    return


def generate_launch_description():
    # Populate launch description with launch arguments
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_interface",
                default_value="mock",
                # Must match the xacro file options for which plugin to load
                choices=["mock", "spot-sdk"],
                description="Hardware interface to load.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Path to general configuration file optionally containing login information.",
            ),
            DeclareLaunchArgument(
                "controllers_config",
                default_value="",
                description=(
                    "Configuration file for the controllers loaded. If not set, a default config file containing a"
                    " forward position controller and a joint state publisher will be loaded, with the appropriate"
                    " configuration based on whether or not the robot has an arm."
                ),
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="forward_position_controller",
                choices=["forward_position_controller"],
                description="Robot controller to start. Must match an entry in controller_config.",
            ),
            DeclareLaunchArgument(
                "mock_has_arm",
                default_value="false",
                choices=["True", "true", "False", "false"],
                description="If in hardware_interface:=mock mode, whether or not the mocked robot has an arm.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description="Flag to enable rviz.",
            ),
        ]
    )
    # Add nodes to launch description
    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))
    return ld
