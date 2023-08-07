from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file", default="")
    config_file_arg = DeclareLaunchArgument(
        "config_file", description="Path to configuration file for the driver.", default_value=""
    )

    has_arm = LaunchConfiguration("has_arm")
    has_arm_arg = DeclareLaunchArgument("has_arm", description="Whether spot has arm", default_value="False")

    ld = LaunchDescription([config_file_arg, has_arm_arg])

    spot_driver_with_namespace_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([ThisLaunchFileDir(), "spot_driver_with_namespace.launch.py"])]
        ),
        launch_arguments={"config_file": config_file, "has_arm": has_arm, "spot_name": ""},
    )

    ld.add_action(spot_driver_with_namespace_launch)

    return ld
