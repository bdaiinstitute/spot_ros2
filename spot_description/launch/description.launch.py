import os
import typing

import launch
import launch_ros
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

pkg_share = launch_ros.substitutions.FindPackageShare(package="spot_description").find("spot_description")
default_model_path = os.path.join(pkg_share, "urdf/spot.urdf.xacro")
default_rviz2_path = os.path.join(pkg_share, "rviz/viz_spot.rviz")


def launch_setup(context: launch.LaunchContext) -> typing.List[launch_ros.actions.Node]:
    namespace = LaunchConfiguration("namespace").perform(context)

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "spot.urdf.xacro"]),
            " ",
            "arm:=",
            LaunchConfiguration("arm"),
            " ",
            "tf_prefix:=",
            LaunchConfiguration("tf_prefix"),
            " ",
        ]
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        namespace=namespace,
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace,
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
        namespace=namespace,
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d" + default_rviz2_path],
    )

    return [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description() -> launch.LaunchDescription:
    launch_arguments = [
        launch.actions.DeclareLaunchArgument(
            name="gui", default_value="True", description="Flag to enable joint_state_publisher_gui"
        ),
        launch.actions.DeclareLaunchArgument(
            name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
        ),
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig", default_value=default_rviz2_path, description="Absolute path to rviz config file"
        ),
        launch.actions.DeclareLaunchArgument("arm", default_value="false", description="include arm in robot model"),
        launch.actions.DeclareLaunchArgument(
            "tf_prefix", default_value='""', description="apply namespace prefix to robot links and joints"
        ),
        launch.actions.DeclareLaunchArgument("namespace", default_value="", description="Namespace for robot tf topic"),
    ]

    return launch.LaunchDescription(launch_arguments + [launch.actions.OpaqueFunction(function=launch_setup)])
