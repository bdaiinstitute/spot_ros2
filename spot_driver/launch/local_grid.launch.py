import launch
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

THIS_PACKAGE = "spot_driver"


def generate_launch_description() -> launch.LaunchDescription:
    # Define launch arguments
    local_grid_name = DeclareLaunchArgument(
        "local_grid_name", default_value="obstacle_distance", description="Name of the local_grid you want published"
    )

    spot_name = DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot")

    local_grid_topic = PathJoinSubstitution([LaunchConfiguration("spot_name"), LaunchConfiguration("local_grid_name")])

    local_grid_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_local_grid_publisher_node",
        output="screen",
        parameters=[{"local_grid_name": LaunchConfiguration("local_grid_name")}],
        namespace=LaunchConfiguration("spot_name"),
        remappings=[("grid_topic_REMAP_ME", local_grid_topic)],
    )

    return launch.LaunchDescription([local_grid_name, spot_name, local_grid_node])
