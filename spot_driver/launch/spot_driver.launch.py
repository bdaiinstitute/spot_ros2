import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    frame_prefix = LaunchConfiguration('frame_prefix')

    frame_prefix_arg = DeclareLaunchArgument('frame_prefix',
                                             description='Frame prefix for robot state publisher, must include /',
                                             default_value='')

    pkg_share = FindPackageShare('spot_description').find('spot_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'spot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    config_ros = os.path.join(
        get_package_share_directory('spot_driver'),
        'config',
        'spot_ros.yaml'
    )
    spot_driver_node = launch_ros.actions.Node(
        package='spot_driver',
        executable='spot_ros2',
        name='spot_ros2',
        output='screen',
        parameters=[config_ros]
    )

    params = {'robot_description': robot_desc, 'frame_prefix': frame_prefix}
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    return launch.LaunchDescription([
        frame_prefix_arg,
        spot_driver_node,
        robot_state_publisher,
    ])
