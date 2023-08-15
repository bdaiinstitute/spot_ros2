# Copyright [2023] Boston Dynamics AI Institute, Inc.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch_ros
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    ThisLaunchFile,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, 
    ThisLaunchFileDir, 
    PathJoinSubstitution, 
    FindExecutable,
    Command
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare("spot_description").find("spot_description")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", 
            PathJoinSubstitution(
                [pkg_share, "urdf", "spot_with_arm.urdf.xacro"]
            ),
            " ",
        ]
    )

    spot_image_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/spot_image_publishers.launch.py']),
        launch_arguments={}.items(),
        condition=IfCondition(),
    ),

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters={"robot_description": robot_description}.items(),
    )

    rosbag_node = launch_ros.actions.Node(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'image_data', '/turtle1/cmd_vel', '/turtle1/pose'],
            output='screen'
        )
    )

    nodes = [
        spot_image_publishers,
        joint_state_publisher_node,
    ]

    return LaunchDescription(nodes)