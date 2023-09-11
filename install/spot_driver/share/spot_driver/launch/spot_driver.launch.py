import os

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def create_rviz_config(robot_name: str) -> None:
    """Writes a configuration file for rviz to visualize a single spot robot"""
    PACKAGE = "spot_driver"

    RVIZ_TEMPLATE_FILENAME = os.path.join(get_package_share_directory(PACKAGE), "rviz", "spot_template.yaml")
    RVIZ_OUTPUT_FILENAME = os.path.join(get_package_share_directory(PACKAGE), "rviz", "spot.rviz")

    with open(RVIZ_TEMPLATE_FILENAME, "r") as template_file:
        config = yaml.safe_load(template_file)
        # Add robot models for each robot
        for display in config["Visualization Manager"]["Displays"]:
            if robot_name != "" and "RobotModel" in display["Class"]:
                display["Description Topic"]["Value"] = f"/{robot_name}/robot_description"

    with open(RVIZ_OUTPUT_FILENAME, "w") as out_file:
        yaml.dump(config, out_file)


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file")
    has_arm = LaunchConfiguration("has_arm")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_filename = LaunchConfiguration("rviz_config_filename").perform(context)
    spot_name = LaunchConfiguration("spot_name").perform(context)

    pkg_share = FindPackageShare("spot_description").find("spot_description")

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        parameters=[config_file, {"spot_name": spot_name}],
        namespace=spot_name,
    )
    ld.add_action(spot_driver_node)

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "spot.urdf.xacro"]),
            " ",
            "arm:=",
            has_arm,
            " ",
        ]
    )

    params = {"robot_description": robot_description}
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
        namespace=spot_name,
    )
    ld.add_action(robot_state_publisher)

    if not rviz_config_filename:
        create_rviz_config(spot_name)
        rviz_config_file = PathJoinSubstitution([FindPackageShare("spot_driver"), "rviz", "spot.rviz"])
    else:
        rviz_config_file = PathJoinSubstitution([FindPackageShare("spot_driver"), "rviz", rviz_config_filename])

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file.perform(context)],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    ld.add_action(rviz)


def generate_launch_description() -> launch.LaunchDescription:
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to configuration file for the driver.",
        )
    )

    launch_args.append(DeclareLaunchArgument("has_arm", default_value="False", description="Whether spot has arm"))

    launch_args.append(DeclareLaunchArgument("launch_rviz", default_value="False", description="Launch RViz?"))
    launch_args.append(
        DeclareLaunchArgument(
            "rviz_config_filename",
            default_value="",
            description="RViz config file name",
        )
    )

    launch_args.append(DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot"))
    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
