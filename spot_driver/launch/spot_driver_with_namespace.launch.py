import os

import launch
import launch_ros
import xacro
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_robot_state_publisher(
    context: LaunchContext, spot_name: LaunchConfiguration, has_arm: LaunchConfiguration, ld: LaunchDescription
) -> None:
    pkg_share = FindPackageShare("spot_description").find("spot_description")
    urdf_dir = os.path.join(pkg_share, "urdf")

    has_arm = has_arm.perform(context) == "True"
    if has_arm:
        xacro_file = os.path.join(urdf_dir, "spot_with_arm.urdf.xacro")
    else:
        xacro_file = os.path.join(urdf_dir, "spot.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")

    params = {"robot_description": robot_desc, "frame_prefix": PathJoinSubstitution([spot_name, ""])}
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=spot_name,
        parameters=[params],
    )
    ld.add_action(robot_state_publisher)


def generate_launch_description() -> launch.LaunchDescription:
    spot_name = LaunchConfiguration("spot_name")
    spot_name_arg = DeclareLaunchArgument("spot_name", description="Name of spot")

    has_arm = LaunchConfiguration("has_arm")
    has_arm_arg = DeclareLaunchArgument("has_arm", description="Whether spot has arm", default_value="False")

    config_file = LaunchConfiguration("config_file")
    config_file_arg = DeclareLaunchArgument("config_file", description="Path to configuration file for the driver.")

    ld = launch.LaunchDescription([spot_name_arg, config_file_arg, has_arm_arg])

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        namespace=spot_name,
        parameters=[config_file, {"spot_name": spot_name}],
    )
    ld.add_action(spot_driver_node)

    ld.add_action(OpaqueFunction(function=launch_robot_state_publisher, args=[spot_name, has_arm, ld]))
    return ld
