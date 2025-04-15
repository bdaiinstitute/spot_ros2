import argparse
import logging
import math
from typing import List, Optional

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.service import Serviced
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import fqn, namespace_with

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander


class SpotInCircle:
    def __init__(
        self,
        robot_name: Optional[str] = None,
        radius: float = 1.2,
        steps: int = 12,
        node: Optional[Node] = None,
    ) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        self.node = node
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self._robot_name = robot_name
        self._radius = radius
        self._steps = steps
        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._odom_frame_name = namespace_with(self._robot_name, ODOM_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )

    def initialize_robot(self) -> bool:
        """Claim and power on the robot, then make it stand."""
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot: " + result.message)
            return False

        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot: " + result.message)
            return False

        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result.success:
            self._logger.error("Robot did not stand: " + result.message)
            return False

        self._logger.info("Successfully stood up.")

        self._get_spot_parameters: Serviced[GetParameters.Request, GetParameters.Response] = Serviced(
            GetParameters, namespace_with(self._robot_name, "spot_ros2/get_parameters"), node=self.node
        )

        if not self._get_spot_parameters.wait_for_service(timeout_sec=5.0):
            self._logger.error(f"No {self._robot_name} driver found, assuming there is no arm")
            self.use_arm = False

        else:
            response = self._get_spot_parameters(GetParameters.Request(names=["has_arm"]), timeout_sec=5.0)
            param = response.values[0]
            self.use_arm = param.bool_value
            if self.use_arm:
                self._logger.info("Arm is available, the arm will be used to gaze at the center of the circle")
            else:
                self._logger.info("Arm is not available")

        return True

    def get_me_a_circle(self, radius: float = -1.2, steps: int = 12) -> List[List[float]]:
        """
        Compute relative movement steps (dx, dy, dyaw) to form a circular trajectory.

        Args:
            radius (float): Radius of the circle in meters.
            steps (int): Number of discrete movements to complete the circle.

        Returns:
            List[List[float]]: Three lists representing dx, dy, dyaw for each step in the robot's body frame.
        """
        dx_all = []
        dy_all = []
        dyaw_all = []

        angle_subtended = 2 * math.pi / steps

        # Negative radius indicates clockwise movement, thus the angle is negative.
        rotation_angle = -angle_subtended if radius < 0 else angle_subtended

        chord_length = 2 * abs(radius) * math.sin(angle_subtended / 2)

        for _ in range(steps):
            dx_all.append(chord_length)
            dy_all.append(0.0)
            dyaw_all.append(rotation_angle)

        return [dx_all, dy_all, dyaw_all]

    def gaze_at_center(self, gaze_target_in_odom: SE3Pose) -> None:
        """
        Command the robot's arm to point at a target position in odom frame.

        Args:
            gaze_target_in_odom (SE3Pose): The 3D pose in odom frame to gaze at.
        """
        x, y, z = gaze_target_in_odom.x, gaze_target_in_odom.y, gaze_target_in_odom.z
        self._logger.info(f"Gazing at point: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        gaze_cmd = RobotCommandBuilder.arm_gaze_command(x, y, z, ODOM_FRAME_NAME)
        gripper_cmd = RobotCommandBuilder.claw_gripper_open_command()
        gaze_robot_cmd = RobotCommandBuilder.build_synchro_command(gripper_cmd, gaze_cmd)

        action_goal = RobotCommand.Goal()

        # Convert from ROS 2 to protobuf
        convert(gaze_robot_cmd, action_goal.command)
        self._robot_command_client.send_goal_and_wait("gaze", action_goal)

    def base_movement(self, dx: float, dy: float, dyaw: float) -> None:
        """
        Move the robot's base by dx, dy, and dyaw relative to current pose (body frame).

        Args:
            dx (float): Forward movement in meters.
            dy (float): Lateral movement in meters.
            dyaw (float): Rotation in radians.
        """
        self._logger.info("Moving the base")

        body_tform_goal = SE2Pose(x=dx, y=dy, angle=dyaw)
        odom_t_robot = self._tf_listener.lookup_a_tform_b(self._odom_frame_name, self._body_frame_name)
        odom_t_robot_se2 = SE3Pose(
            odom_t_robot.transform.translation.x,
            odom_t_robot.transform.translation.y,
            odom_t_robot.transform.translation.z,
            Quat(
                odom_t_robot.transform.rotation.w,
                odom_t_robot.transform.rotation.x,
                odom_t_robot.transform.rotation.y,
                odom_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()

        odom_t_goal = odom_t_robot_se2 * body_tform_goal

        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=odom_t_goal.x,
            goal_y=odom_t_goal.y,
            goal_heading=odom_t_goal.angle,
            frame_name=ODOM_FRAME_NAME,
        )
        action_goal = RobotCommand.Goal()

        # Convert from ROS 2 to protobuf
        convert(proto_goal, action_goal.command)
        self._robot_command_client.send_goal_and_wait("walk_forward", action_goal)
        self._logger.info("Successfully walked forward")

    def circle(self) -> None:
        """
        Moves Spot in a circle while gazing at a fixed point in the center.
        In this example, the robot moves clockwise in a circle with a radius of 1.2 meters, in 12 steps.
        """
        self._logger.info("Starting circular motion")

        dx_all, dy_all, dyaw_all = self.get_me_a_circle(self._radius, self._steps)

        if self.use_arm:
            # Convert target from body frame to odom frame
            hand_pos_rt_body = geometry_pb2.Vec3(x=0.0, y=self._radius, z=0.1)
            body_Q_hand = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)
            body_t_hand = geometry_pb2.SE3Pose(position=hand_pos_rt_body, rotation=body_Q_hand)

            body_to_odom = self._tf_listener.lookup_a_tform_b(self._odom_frame_name, self._body_frame_name)
            odom_T_body_se3 = math_helpers.SE3Pose(
                body_to_odom.transform.translation.x,
                body_to_odom.transform.translation.y,
                body_to_odom.transform.translation.z,
                math_helpers.Quat(
                    body_to_odom.transform.rotation.w,
                    body_to_odom.transform.rotation.x,
                    body_to_odom.transform.rotation.y,
                    body_to_odom.transform.rotation.z,
                ),
            )
            gaze_target_in_odom = odom_T_body_se3 * math_helpers.SE3Pose.from_proto(body_t_hand)

        for i in range(len(dx_all)):
            try:
                if self.use_arm:
                    self.gaze_at_center(gaze_target_in_odom)
                self.base_movement(dx_all[i], dy_all[i], dyaw_all[i])
            finally:
                result = self._robot.command("stop")
                if not result.success:
                    self._logger.error("Unable to make the robot stop: " + result.message)

        if self.use_arm:
            result = self._robot.command("arm_stow")
            if not result.success:
                self._logger.error("Unable to stow the arm: " + result.message)
        result = self._robot.command("sit")
        if not result.success:
            self._logger.error("Unable to sit the robot: " + result.message)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        help="Name of the robot",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=-1.2,
        help="Radius of the circle  in meters (negative radius value means clockwise circle, and vice versa)",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=12,
        help="Number of steps to complete a circle (default: 12)",
    )
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    goto = SpotInCircle(args.robot, args.radius, args.steps, main.node)
    result = goto.initialize_robot()
    if result:
        goto.circle()
    return 0


if __name__ == "__main__":
    exit(main())
