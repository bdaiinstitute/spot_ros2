from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    has_arm_arg = DeclareLaunchArgument(
        "has_arm",
        default_value="false",
        choices=["true", "false"],
        description="Whether the robot has an arm",
    )
    controllers_config_arg = DeclareLaunchArgument(
        "controllers_config",
        default_value="spot_controllers_without_arm.yaml",
        description="YAML file for configuring the controllers.",
    )
    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value="spot.urdf.xacro",
        description="URDF/XACRO description file with the robot.",
    )
    robot_controller_arg = DeclareLaunchArgument(
        "robot_controller",
        default_value="forward_position_controller",
        # Currently this is the only option we allow, but more could be included by adding to the controllers_file
        choices=["forward_position_controller"],
        description="Robot controller to start.",
    )

    # Generate the robot description
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("spot_ros2_control"), "xacro", LaunchConfiguration("description_file")]
            ),
            " has_arm:=",
            LaunchConfiguration("has_arm"),
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    # Configuration files
    controller_config_file = PathJoinSubstitution(
        [FindPackageShare("spot_ros2_control"), "config", LaunchConfiguration("controllers_config")]
    )
    rviz_config_file = PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "rviz", "spot_ros2_control.rviz"])

    # Nodes
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controller_config_file],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    robot_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("robot_controller"), "-c", "/controller_manager"],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            has_arm_arg,
            controllers_config_arg,
            description_file_arg,
            robot_controller_arg,
            ros2_control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner_node,
            robot_controller_spawner_node,
            rviz_node,
        ]
    )
