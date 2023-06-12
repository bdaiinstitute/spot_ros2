# Copyright [2023] Boston Dynamics AI Institute, Inc.

import launch
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> launch.LaunchDescription:
    spot_name = LaunchConfiguration("spot_name", default="")
    spot_name_arg = DeclareLaunchArgument("spot_name", description="Name of spot", default_value="")

    image_publish_rate = LaunchConfiguration("image_publish_rate")
    image_publish_rate_arg = DeclareLaunchArgument(
        "image_publish_rate",
        description="Rate in (hz) of image publishing",
        default_value="10",
    )

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

    nodes = [spot_name_arg, image_publish_rate_arg, publish_rgb_arg, publish_depth_arg, publish_depth_registered_arg]
    camera_types = ["camera", "depth", "depth_registered"]
    publish_camera_types = [publish_rgb, publish_depth, publish_depth_registered]
    for camera_type, publish_camera_type in zip(camera_types, publish_camera_types):
        camera_node = launch_ros.actions.Node(
            package="spot_driver",
            executable="spot_publish_cameras",
            name=f"spot_image_publisher_{camera_type}",
            output="screen",
            namespace=spot_name,
            parameters=[
                {
                    "spot_name": spot_name,
                    "image_publish_rate": image_publish_rate,
                    "camera_type": camera_type,
                }
            ],
            condition=IfCondition(publish_camera_type),
        )
        nodes.append(camera_node)

    return launch.LaunchDescription(nodes)
