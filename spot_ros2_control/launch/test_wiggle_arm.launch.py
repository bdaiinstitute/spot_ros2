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
                    "has_arm": "true",
                    "controllers_config": "spot_controllers_with_arm.yaml",
                    "robot_controller": "forward_position_controller",
                }.items(),
            ),
            Node(package="spot_ros2_control", executable="wiggle_arm", name="wiggle_arm", output="screen"),
        ]
    )
