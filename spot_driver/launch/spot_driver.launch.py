import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():

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

    return launch.LaunchDescription([
        spot_driver_node
    ])