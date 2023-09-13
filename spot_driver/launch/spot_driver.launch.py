import os

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

THIS_PACKAGE = "spot_driver"


def create_rviz_config(robot_name: str) -> None:
    """Writes a configuration file for rviz to visualize a single spot robot"""

    RVIZ_TEMPLATE_FILENAME = os.path.join(get_package_share_directory(THIS_PACKAGE), "rviz", "spot_template.yaml")
    RVIZ_OUTPUT_FILENAME = os.path.join(get_package_share_directory(THIS_PACKAGE), "rviz", "spot.rviz")

    with open(RVIZ_TEMPLATE_FILENAME, "r") as template_file:
        config = yaml.safe_load(template_file)

        if robot_name:
            # replace fixed frame with robot body frame
            config["Visualization Manager"]["Global Options"]["Fixed Frame"] = f"{robot_name}/vision"
            # Add robot models for each robot
            for display in config["Visualization Manager"]["Displays"]:
                if "RobotModel" in display["Class"]:
                    display["Description Topic"]["Value"] = f"/{robot_name}/robot_description"

                if "Image" in display["Class"]:
                    topic_name = display["Topic"]["Value"]
                    display["Topic"]["Value"] = f"/{robot_name}{topic_name}"

    with open(RVIZ_OUTPUT_FILENAME, "w") as out_file:
        yaml.dump(config, out_file)


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file")
    has_arm = LaunchConfiguration("has_arm")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    spot_name = LaunchConfiguration("spot_name").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    use_depth_registered_nodelets = LaunchConfiguration("use_depth_registered_nodelets")

    pkg_share = FindPackageShare("spot_description").find("spot_description")

    spot_driver_params = [config_file, {"spot_name": spot_name}]
    if use_depth_registered_nodelets.perform(context):
        spot_driver_params.append({"publish_depth_registered": False})

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        parameters=spot_driver_params,
        namespace=spot_name,
    )
    ld.add_action(spot_driver_node)

    if not tf_prefix and spot_name:
        tf_prefix = PathJoinSubstitution([spot_name, ""])

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "spot.urdf.xacro"]),
            " ",
            "arm:=",
            has_arm,
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

    # It looks like passing an optional of value "None" gets converted to a string of value "None"
    if not rviz_config_file or rviz_config_file == "None":
        create_rviz_config(spot_name)
        rviz_config_file = PathJoinSubstitution([FindPackageShare(THIS_PACKAGE), "rviz", "spot.rviz"]).perform(context)

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    ld.add_action(rviz)

    registered_depth_image_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare(THIS_PACKAGE), "launch", "spot_depth_registered_publishers.launch.py"]
                )
            ]
        ),
        launch_arguments={"spot_name": spot_name, "has_arm": has_arm}.items(),
        condition=IfCondition(use_depth_registered_nodelets),
    )
    ld.add_action(registered_depth_image_nodes)


def generate_launch_description() -> launch.LaunchDescription:
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to configuration file for the driver.",
        )
    )
    launch_args.append(DeclareLaunchArgument("has_arm", default_value="False", description="Whether spot has arm"))
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
            "use_depth_registered_nodelets",
            default_value="True",
            description="launch composable nodes for publishing depth registered images.",
        )
    )
    launch_args.append(DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot"))

    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
