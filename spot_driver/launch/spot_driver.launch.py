import launch
import launch_ros
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


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file")
    has_arm = LaunchConfiguration("has_arm")
    launch_rviz = LaunchConfiguration("launch_rviz")
    tf_prefix = LaunchConfiguration("tf_prefix")
    parent = LaunchConfiguration("parent")

    pkg_share = FindPackageShare("spot_description").find("spot_description")

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver", executable="spot_ros2", name="spot_ros2", output="screen", parameters=[config_file]
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
            "tf_prefix:=",
            tf_prefix,
            " ",
            "parent:=",
            parent,
            " ",
        ]
    )

    params = {"robot_description": robot_description}
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )
    ld.add_action(robot_state_publisher)

    rviz_config_file = PathJoinSubstitution([FindPackageShare("spot_driver"), "rviz", "viz_spot.rviz"])

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file, "-f", parent.perform(context)],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    ld.add_action(rviz)


def generate_launch_description() -> launch.LaunchDescription:
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument("config_file", default_value="", description="Path to configuration file for the driver.")
    )

    launch_args.append(DeclareLaunchArgument("has_arm", default_value="False", description="Whether spot has arm"))

    launch_args.append(
        DeclareLaunchArgument(
            "tf_prefix", default_value='""', description="apply namespace prefix to robot links and joints"
        )
    )

    launch_args.append(DeclareLaunchArgument("parent", default_value="world", description="parent link/frame for Spot"))

    launch_args.append(DeclareLaunchArgument("launch_rviz", default_value="False", description="Launch RViz?"))

    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, ld=ld))

    return ld
