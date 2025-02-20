import logging
from typing import Optional

import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from geometry_msgs.msg import Pose
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import fqn, namespace_with

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander

# Where we want the robot to walk to relative to itself


class RobotCommander:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        self._node = node or ros_scope.node()
        if self._node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self._robot_name = robot_name
        self._logger.info(f"{self._robot_name}")

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(self._node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(robot_name=self._robot_name, node=self._node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), self._node
        )

    def initialize_robot(self) -> bool:
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

    def walk_forward_with_vision_frame_goal(self, waypoint: Pose) -> bool:
        """
        Walk forward to a goal in the world frame.
        Arguments:
            waypoint: The waypoint to walk to with respect to the body frame.
                      Type is geometry_msgs.msg.Pose.
                      This is using the vision frame of the spot
        """
        self._logger.info("Walking")
        world_t_robot = self._tf_listener.lookup_a_tform_b(self._vision_frame_name, self._body_frame_name)
        world_t_robot_se2 = SE3Pose(
            world_t_robot.transform.translation.x,
            world_t_robot.transform.translation.y,
            world_t_robot.transform.translation.z,
            Quat(
                world_t_robot.transform.rotation.w,
                world_t_robot.transform.rotation.x,
                world_t_robot.transform.rotation.y,
                world_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()

        goal = SE3Pose(
            waypoint.position.x,
            waypoint.position.y,
            waypoint.position.z,
            Quat(
                waypoint.orientation.w,
                waypoint.orientation.x,
                waypoint.orientation.y,
                waypoint.orientation.z,
            ),
        ).get_closest_se2_transform()
        world_t_goal = world_t_robot_se2 * goal
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=world_t_goal.x,
            goal_y=world_t_goal.y,
            goal_heading=world_t_goal.angle,
            frame_name=VISION_FRAME_NAME,  # use Boston Dynamics' frame conventions
        )
        action_goal = RobotCommand.Goal()
        convert(proto_goal, action_goal.command)
        result = self._robot_command_client.send_goal_and_wait("walk_forward", action_goal, timeout_sec=5)
        if result is not None:
            self._logger.info(f"Result: {result.message}")
            self._logger.info(f"Navigated to waypoint: {waypoint}")
        else:
            self._logger.info(f"Failed to navigate to waypoint: {waypoint}")
        return result
