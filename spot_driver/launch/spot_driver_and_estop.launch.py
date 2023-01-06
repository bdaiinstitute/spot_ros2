import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spot_name = LaunchConfiguration('spot_name')

    spot_name_arg = DeclareLaunchArgument('spot_name', description='Name of spot')

    spot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('spot_driver'),
                'launch/spot_driver_with_namespace.launch.py'
            ])
        ]),
        launch_arguments={
            'spot_name': spot_name,
        }.items()
    )

    estop = launch_ros.actions.Node(
        package='spot_driver',
        executable='estop_gui.sh',
        output='screen',
        arguments=[spot_name]
    )

    return launch.LaunchDescription([
        spot_name_arg,
        spot_driver,
        estop,
    ])
