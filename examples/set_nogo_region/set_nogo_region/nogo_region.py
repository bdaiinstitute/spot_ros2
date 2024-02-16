import argparse
import logging
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.utilities import fqn, namespace_with
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from rclpy.node import Node
from utilities.simple_spot_commander import SimpleSpotCommander
from utilities.tf_listener_wrapper import TFListenerWrapper

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.srv import ListWorldObjects  # type: ignore

# Where we want the robot to walk to relative to itself
ROBOT_T_GOAL = SE2Pose(1.0, 0.0, 0.0)


class NoGoRegion:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self.node = node
        self._robot_name = robot_name

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)

        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )
        self._list_wo_client = node.create_client(
            ListWorldObjects, namespace_with(self._robot_name, "list_world_objects")
        )

    def initialize_robot(self) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        return True

    def list_current_world_objects(self) -> None:
        request = ListWorldObjects.Request()
        print("Calling list world object")
        future = self._list_wo_client.call_async(request)
        if not wait_for_future(future, context=self.node.context):
            return
        print("Getting result")
        response = future.result()
        for wo in response.response.world_objects:
            print(f"\n\nID: {wo.id} name: {wo.name}\n{wo}")


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    nogo = NoGoRegion(args.robot, main.node)
    nogo.initialize_robot()
    nogo.list_current_world_objects()
    return 0


if __name__ == "__main__":
    exit(main())
