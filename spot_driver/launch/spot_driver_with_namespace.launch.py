import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spot_name = LaunchConfiguration('spot_name')

    spot_name_arg = DeclareLaunchArgument('spot_name', description='Name of spot')

    spot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('spot_driver'),
                'launch/spot_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'frame_prefix': PathJoinSubstitution([spot_name, '']),
        }.items()
    )

    spot_driver_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(spot_name),
            spot_driver,
        ]
    )

    return launch.LaunchDescription([
        spot_name_arg,
        spot_driver_with_namespace,
    ])
