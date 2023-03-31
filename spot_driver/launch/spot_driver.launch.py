import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    config_file = LaunchConfiguration('config_file', default='')
    config_file_arg = DeclareLaunchArgument('config_file', description='Path to configuration file for the driver.',
                                            default_value='')

    pkg_share = FindPackageShare('spot_description').find('spot_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'spot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    publish_rgb = LaunchConfiguration("publish_rgb", default="true")
    publish_rgb_arg = DeclareLaunchArgument(
        "publish_rgb",
        description="Start publishing all RGB channels on Spot cameras",
        default_value="true",
    )

    publish_depth = LaunchConfiguration("publish_depth", default="true")
    publish_depth_arg = DeclareLaunchArgument(
        "publish_depth",
        description="Start publishing all depth channels on Spot cameras",
        default_value="true",
    )

    publish_depth_registered = LaunchConfiguration("publish_depth_registered", default="false")
    publish_depth_registered_arg = DeclareLaunchArgument(
        "publish_depth_registered",
        description="Start publishing all depth_registered channels on Spot cameras",
        default_value="false",
    )

    driver_params = {
        'publish_rgb': publish_rgb,
        'publish_depth': publish_depth,
        'publish_depth_registered': publish_depth_registered
    }
    spot_driver_node = launch_ros.actions.Node(
        package='spot_driver',
        executable='spot_ros2',
        name='spot_ros2',
        output='screen',
        parameters=[config_file, driver_params]
    )

    params = {'robot_description': robot_desc}
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    return launch.LaunchDescription([
        publish_rgb_arg,
        publish_depth_arg,
        publish_depth_registered_arg,
        config_file_arg,
        spot_driver_node,
        robot_state_publisher,
    ])
