# Copyright [2023] Boston Dynamics AI Institute, Inc.

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros


# camera_sources = ["frontleft", "frontright", "left", "right", "back"]
# camera_types = ["camera", "depth", "depth_registered"]

camera_sources = ["frontleft"]
camera_types = ["camera"]


def generate_launch_description():
    spot_name = LaunchConfiguration('spot_name')
    spot_name_arg = DeclareLaunchArgument('spot_name', description='Name of spot')

    image_publish_rate = LaunchConfiguration("image_publish_rate")
    image_publish_rate_arg = DeclareLaunchArgument(
        "image_publish_rate",
        description='Rate in (hz) of image publishing',
        default_value="10",
    )

    nodes = [spot_name_arg, image_publish_rate_arg]
    camera_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="publish_camera",
        name=f"image_publisher",
        output="screen",
        namespace=spot_name,
        parameters=[
            {
                "spot_name": spot_name,
                "image_publish_rate": image_publish_rate,
            }
        ]
    )
    nodes.append(camera_node)

    return launch.LaunchDescription(nodes)
