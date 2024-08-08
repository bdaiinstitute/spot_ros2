from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("spot_ros2_control"), "launch", "spot_ros2_control.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "has_arm": "false",
                    "controllers_config": "spot_controllers_without_arm.yaml",
                    "robot_controller": "forward_position_controller",
                    "hardware_interface": "mock",
                }.items(),
            ),
            Node(
                package="spot_ros2_control",
                executable="noarm_squat",
                name="noarm_squat",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "config", "examples.yaml"])],
            ),
        ]
    )
