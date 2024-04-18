# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
import os
from enum import Enum
from typing import List, Optional, Tuple

import launch
import launch_ros
import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

from spot_wrapper.wrapper import SpotWrapper

THIS_PACKAGE = "spot_driver"


class DepthRegisteredMode(Enum):
    DISABLE = (0,)
    FROM_SPOT = (1,)
    FROM_NODELETS = (2,)


def get_camera_sources(has_arm: bool) -> List[str]:
    camera_sources = ["frontleft", "frontright", "left", "right", "back"]
    if has_arm:
        camera_sources.append("hand")
    return camera_sources


def create_depth_registration_nodelets(
    context: launch.LaunchContext,
    spot_name: LaunchConfiguration,
    has_arm: bool,
) -> List[launch_ros.descriptions.ComposableNode]:
    """Create the list of depth_image_proc::RegisterNode composable nodes required to generate registered depth images
    for Spot's cameras."""

    composable_node_descriptions = []

    for camera in get_camera_sources(has_arm):
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
    return composable_node_descriptions


def create_point_cloud_nodelets(
    context: launch.LaunchContext,
    spot_name: LaunchConfiguration,
    has_arm: bool,
) -> List[launch_ros.descriptions.ComposableNode]:
    """Create the list of depth_image_proc::PointCloudXyzrgbNode composable nodes required to generate point clouds for
    each pair of RGB and registered depth cameras."""

    composable_node_descriptions = []

    for camera in get_camera_sources(has_arm):
        composable_node_descriptions.append(
            launch_ros.descriptions.ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb_node_" + camera,
                namespace=spot_name,
                # Each entry in the remappings list is a tuple.
                # The first element in the tuple is the internal name of the topic used within the nodelet.
                # The second element is the external name of the topic used by other nodes in the system.
                remappings=[
                    ("rgb/camera_info", PathJoinSubstitution(["camera", camera, "camera_info"]).perform(context)),
                    ("rgb/image_rect_color", PathJoinSubstitution(["camera", camera, "image"]).perform(context)),
                    (
                        "depth_registered/image_rect",
                        PathJoinSubstitution(["depth_registered", camera, "image"]).perform(context),
                    ),
                    ("points", PathJoinSubstitution(["depth_registered", camera, "points"]).perform(context)),
                ],
            ),
        )
    return composable_node_descriptions


def get_login_parameters(context: LaunchContext) -> Tuple[str, str, str, Optional[int], Optional[str]]:
    """Obtain the username, password, hostname, and port of Spot from the environment variables or, if they are not
    set, the configuration file yaml."""
    # Get value from environment variables
    username = os.getenv("BOSDYN_CLIENT_USERNAME")
    password = os.getenv("BOSDYN_CLIENT_PASSWORD")
    hostname = os.getenv("SPOT_IP")
    portnum = os.getenv("SPOT_PORT")
    port = int(portnum) if portnum else None
    certificate = os.getenv("SPOT_CERTIFICATE")

    # parse the yaml to determine if login information is set there
    config_file_path = LaunchConfiguration("config_file").perform(context)
    if os.path.isfile(config_file_path):
        with open(config_file_path, "r") as config_yaml:
            try:
                config_dict = yaml.safe_load(config_yaml)
                if ("/**" in config_dict) and ("ros__parameters" in config_dict["/**"]):
                    ros_params = config_dict["/**"]["ros__parameters"]
                    # only set username/password/hostname if they were not already set as environment variables.
                    if (not username) and ("username" in ros_params):
                        username = ros_params["username"]
                    if (not password) and ("password" in ros_params):
                        password = ros_params["password"]
                    if (not hostname) and ("hostname" in ros_params):
                        hostname = ros_params["hostname"]
                    if not port and "port" in ros_params:
                        port = ros_params["port"]
                    if not certificate and "certificate" in ros_params:
                        certificate = ros_params["certificate"]
            except yaml.YAMLError as exc:
                print("Parsing config_file yaml failed with: {}".format(exc))
    if (not username) or (not password) or (not hostname):
        raise ValueError(
            "One or more of your login credentials has not been specified! Got invalid values of "
            "[Username: '{}' Password: '{}' Hostname: '{}']. Ensure that your environment variables are set or "
            "update your config_file yaml.".format(username, password, hostname)
        )
    return username, password, hostname, port, certificate


def spot_has_arm(context: LaunchContext) -> bool:
    """Check if Spot has an arm by logging in and instantiating a SpotWrapper"""
    spot_name = LaunchConfiguration("spot_name").perform(context)
    logger = logging.getLogger("spot_driver_launch")
    username, password, hostname, port, certificate = get_login_parameters(context)
    spot_wrapper = SpotWrapper(
        username=username,
        password=password,
        hostname=hostname,
        port=port,
        cert_resource_glob=certificate,
        robot_name=spot_name,
        logger=logger,
    )
    return spot_wrapper.has_arm()


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    spot_name = LaunchConfiguration("spot_name").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    depth_registered_mode_config = LaunchConfiguration("depth_registered_mode")
    publish_point_clouds_config = LaunchConfiguration("publish_point_clouds")
    mock_enable = IfCondition(LaunchConfiguration("mock_enable", default="False")).evaluate(context)

    # if config_file has been set (and is not the default empty string) and is also not a file, do not launch anything.
    config_file_path = config_file.perform(context)
    if (config_file_path != "") and (not os.path.isfile(config_file_path)):
        raise FileNotFoundError("Configuration file '{}' does not exist!".format(config_file_path))

    if mock_enable:
        mock_has_arm = IfCondition(LaunchConfiguration("mock_has_arm")).evaluate(context)
        has_arm = mock_has_arm
    else:
        has_arm = spot_has_arm(context)

    pkg_share = FindPackageShare("spot_description").find("spot_description")

    depth_registered_mode_string = depth_registered_mode_config.perform(context).lower()
    if depth_registered_mode_string == "from_spot":
        depth_registered_mode = DepthRegisteredMode.FROM_SPOT
    elif depth_registered_mode_string == "from_nodelets":
        depth_registered_mode = DepthRegisteredMode.FROM_NODELETS
    elif depth_registered_mode_string == "disable":
        depth_registered_mode = DepthRegisteredMode.DISABLE
    else:
        print(
            "Error: Invalid option `"
            + depth_registered_mode_string
            + "` provided for launch argument `depth_registered_mode`. Must be one of [disable, from_spot,"
            " from_nodelets]."
        )
        return

    publish_point_clouds = True if publish_point_clouds_config.perform(context).lower() == "true" else False
    if depth_registered_mode is DepthRegisteredMode.DISABLE and publish_point_clouds:
        print(
            "Warning: Point cloud publisher nodelets will not be launched because depth_registered_mode is set to"
            " `disable`. Set depth_registered_mode to `from_nodelets` or `from_spot` to enable point cloud publishing."
        )
        publish_point_clouds = False

    # Since spot_image_publisher_node is responsible for retrieving and publishing images, disable all image publishing
    # in spot_driver.
    spot_driver_params = {
        "spot_name": spot_name,
        "mock_enable": mock_enable,
        "publish_depth_registered": False,
        "publish_depth": False,
        "publish_rgb": False,
    }

    if mock_enable:
        mock_spot_driver_params = {"mock_has_arm": mock_has_arm}
        # Merge the two dicts
        spot_driver_params = {**spot_driver_params, **mock_spot_driver_params}

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        parameters=[config_file, spot_driver_params],
        namespace=spot_name,
    )
    ld.add_action(spot_driver_node)

    spot_image_publisher_params = {
        "spot_name": spot_name,
    }
    # If using nodelets to generate registered depth images, do not retrieve and publish registered depth images using
    # spot_image_publisher_node.
    if depth_registered_mode is not DepthRegisteredMode.FROM_SPOT:
        spot_image_publisher_params.update({"publish_depth_registered": False})

    spot_image_publisher_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_image_publisher_node",
        output="screen",
        parameters=[config_file, spot_image_publisher_params],
        namespace=spot_name,
    )
    ld.add_action(spot_image_publisher_node)

    if not tf_prefix and spot_name:
        tf_prefix = PathJoinSubstitution([spot_name, ""])

    kinematc_node_params = {"spot_name": spot_name}
    kinematic_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_inverse_kinematics_node",
        output="screen",
        parameters=[config_file, kinematc_node_params],
        namespace=spot_name,
    )
    ld.add_action(kinematic_node)

    object_sync_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="object_synchronizer_node",
        output="screen",
        parameters=[config_file, {"spot_name": spot_name}],
        namespace=spot_name,
    )
    ld.add_action(object_sync_node)

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "spot.urdf.xacro"]),
            " ",
            "arm:=",
            TextSubstitution(text=str(has_arm).lower()),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
        ]
    )

    params = {"robot_description": robot_description}
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
        namespace=spot_name,
    )
    ld.add_action(robot_state_publisher)

    spot_robot_state_publisher_params = {"spot_name": spot_name, "preferred_odom_frame": "odom"}
    spot_robot_state_publisher = launch_ros.actions.Node(
        package="spot_driver",
        executable="state_publisher_node",
        output="screen",
        parameters=[config_file, spot_robot_state_publisher_params],
        namespace=spot_name,
    )
    ld.add_action(spot_robot_state_publisher)

    spot_alert_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_alerts",
        name="spot_alerts",
        output="screen",
        namespace=spot_name,
    )
    ld.add_action(spot_alert_node)

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(THIS_PACKAGE), "/launch", "/rviz.launch.py"]),
        launch_arguments={
            "spot_name": spot_name,
            "rviz_config_file": rviz_config_file,
        }.items(),
        condition=IfCondition(launch_rviz),
    )

    ld.add_action(rviz)

    # Parse config options to create a list of composable node descriptions for the nodelets we want to run within the
    # composable node container.
    composable_node_descriptions = (
        create_depth_registration_nodelets(context, spot_name, has_arm)
        if depth_registered_mode is DepthRegisteredMode.FROM_NODELETS
        else []
    ) + (create_point_cloud_nodelets(context, spot_name, has_arm) if publish_point_clouds else [])
    container = launch_ros.actions.ComposableNodeContainer(
        name="container",
        namespace=spot_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=composable_node_descriptions,
    )
    ld.add_action(container)


def generate_launch_description() -> launch.LaunchDescription:
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to configuration file for the driver.",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="apply namespace prefix to robot links and joints",
        )
    )
    launch_args.append(DeclareLaunchArgument("launch_rviz", default_value="False", description="Launch RViz?"))
    launch_args.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description="RViz config file",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "depth_registered_mode",
            default_value="from_nodelets",
            description=(
                "One of [disable, from_spot, from_nodelets]. If `disable` is set, do not publish registered depth"
                " images. If `from_spot` is set, request registered depth images from Spot through its SDK. If"
                " `from_nodelets` is set, use depth_image_proc::RegisterNode component nodes running on the host"
                " computer to create registered depth images (this reduces the computational load on Spot's internal"
                " systems)."
            ),
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "publish_point_clouds",
            default_value="false",
            description=(
                "If true, create and publish point clouds for each depth registered and RGB camera pair. Requires that"
                " the depth_register_mode launch argument is set to a value that is not `disable`."
            ),
        )
    )
    launch_args.append(DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot"))

    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
