# Copyright [2023] Boston Dynamics AI Institute, Inc.


import launch
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_depth_register_nodelets(
    context: launch.LaunchContext,
    spot_name: LaunchConfiguration,
    has_arm: LaunchConfiguration,
    ld: launch.LaunchDescription,
) -> None:
    composable_node_descriptions = []

    camera_sources = ["frontleft", "frontright", "left", "right", "back"]
    if has_arm.perform(context) == "true" or has_arm.perform(context) == "True":
        camera_sources.append("hand")

    for camera in camera_sources:
        composable_node_descriptions.append(
            launch_ros.descriptions.ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_node_" + camera,
                namespace=spot_name,
                # Each entry in the remappings list is a tuple.
                # The first element in the tuple is the internal name of the topic used within the nodelet.
                # The second element is the external name of the topic used by other nodes in the system.
                remappings=[
                    ("depth/image_rect", PathJoinSubstitution(["depth", camera, "image"]).perform(context)),
                    ("depth/camera_info", PathJoinSubstitution(["depth", camera, "camera_info"]).perform(context)),
                    ("rgb/camera_info", PathJoinSubstitution(["camera", camera, "camera_info"]).perform(context)),
                    (
                        "depth_registered/image_rect",
                        PathJoinSubstitution(["depth_registered", camera, "image"]).perform(context),
                    ),
                    (
                        "depth_registered/camera_info",
                        PathJoinSubstitution(["depth_registered", camera, "camera_info"]).perform(context),
                    ),
                ],
            )
        )

    container = launch_ros.actions.ComposableNodeContainer(
        name="container",
        namespace=spot_name,
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=composable_node_descriptions,
    )

    ld.add_action(container)


def generate_launch_description() -> launch.LaunchDescription:
    spot_name = LaunchConfiguration("spot_name")
    spot_name_arg = DeclareLaunchArgument("spot_name", description="Name of spot")

    has_arm = LaunchConfiguration("has_arm")
    has_arm_arg = DeclareLaunchArgument("has_arm", default_value="False", description="Whether spot has arm")

    ld = launch.LaunchDescription([spot_name_arg, has_arm_arg])

    ld.add_action(OpaqueFunction(function=launch_depth_register_nodelets, args=[spot_name, has_arm, ld]))
    return ld
