import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = FindPackageShare('spot_description').find('spot_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'spot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    config_login = os.path.join(
        get_package_share_directory('spot_driver'),
        'config',
        'spot_login.yaml'
    )
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
        parameters=[config_login,config_ros],
    )

    params = {'robot_description': robot_desc}
    robot_state_publisher = launch_ros.actions.Node(
                                package='robot_state_publisher',
                                executable='robot_state_publisher',
                                output='screen',
                                parameters=[params])

    return launch.LaunchDescription([
        spot_driver_node,
        robot_state_publisher
    ])