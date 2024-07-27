import argparse
import logging
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.utilities import fqn, namespace_with
from rclpy.node import Node
import time

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.action import NavigateTo  # type: ignore
from spot_msgs.srv import GraphNavInitialize
from spot_msgs.srv import ListGraph

from spot_examples.simple_spot_commander import SimpleSpotCommander


class NavigateToWaypoint:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._robot_name = robot_name
        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )
        self._robot_navigation_client = ActionClientWrapper(
           NavigateTo, namespace_with(self._robot_name, "navigate_to"), node
        )
        self._grap_nav_client = node.create_client(GraphNavInitialize, "/" + self._robot_name + "/graph_nav_initialize")
        self._waypoint_client = node.create_client(ListGraph, "/" + self._robot_name + "/list_graph")

        while not self._grap_nav_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Service not available, waiting...')

        while not self._waypoint_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Service not available, waiting...')

        self._logger.info('Service Found')

    def initialize_robot(self, args) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")
        # Stand the robot up.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result.success:
            self._logger.error("Robot did not stand message was " + result.message)
            return False
        self._logger.info("Successfully stood up.")

        return True

    def list_waypoints(self, upload_path):
        request = ListGraph.Request()
        request.upload_filepath = upload_path
        response = self._waypoint_client.call(request)
        if len(response.waypoint_ids) > 0:
            self._logger.info("ListGraph successful")
            return response.waypoint_ids
        else:
            self._logger.error("ListGraph failed: " + str(response.waypoint_ids))
            return None
        

    def initialize_graph_nav(self, args):
        request = GraphNavInitialize.Request()
        request.upload_filepath = args.upload_path
        request.initial_localization_fiducial = args.initial_localization_fiducial  
        request.initial_localization_waypoint = args.initial_localization_waypoint

        # Use the node
        response = self._grap_nav_client.call(request)

        if response.success:
            self._logger.info("GraphNavInitialize successful")
        else:
            self._logger.error("GraphNavInitialize failed: " + response.message)
            return False        
        return True

    def navigate_to_waypoint(self, args):
        self._logger.info("Navigate to waypoint")
        goal_msg = NavigateTo.Goal()
        waypoint = args.waypoint_ids[args.navigate_to]
        goal_msg.navigate_to = waypoint.replace("[", "").replace("]", "").replace(" ", "").replace(",", "")
        self._robot_navigation_client.send_goal_and_wait("navigate_to", goal_msg)
       
        self._logger.info("Sent goal")

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="Robot hostname or IP address")
    parser.add_argument("--navigate_to", type=int, default=0, help="Index of the waypoint to navigate to")
    parser.add_argument("--initialize_position", action='store_true', help="Flag to initialize robot's position")
    parser.add_argument("--upload_path", type=str, default="/home/robot/spot_map/downloaded_graph", help="Path to upload the navigation graph")
    parser.add_argument("--initial_localization_fiducial", action='store_true', help="Use fiducial for initial localization")
    parser.add_argument("--initial_localization_waypoint", type=str, default="", help="Waypoint ID for initial localization")
    parser.add_argument("--waypoint_ids", nargs='*', required=True, help="List of waypoint IDs for navigation")

    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    navigate_to_waypoint = NavigateToWaypoint(args.robot, main.node)
    navigate_to_waypoint.initialize_robot(args)
    time.sleep(2)
    position_initialized = True
    if args.initialize_position:
        position_initialized = navigate_to_waypoint.initialize_graph_nav(args)

    if position_initialized:
        navigate_to_waypoint.navigate_to_waypoint(args)
    return 0


if __name__ == "__main__":
    exit(main())
