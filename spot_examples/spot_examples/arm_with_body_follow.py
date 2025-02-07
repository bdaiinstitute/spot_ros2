import argparse
import time
from typing import Optional

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander


class ArmWithBodyFollow:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self.node = ros_scope.node()
        if self.node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self.logger = self.node.get_logger()

        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.tf_listener = TFListenerWrapper(node)
        self.tf_listener.wait_for_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        self.robot = SimpleSpotCommander(robot_name, node)
        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)

    def initialize_robot(self) -> bool:
        # Claim robot
        self.logger.info("Claiming robot")
        result = self.robot.command("claim")
        if not result.success:
            self.node.get_logger().error("Unable to claim robot message was " + result.message)
            return False
        self.logger.info("Claimed robot")

        # Power on robot
        self.logger.info("Powering on robot")
        result = self.robot.command("power_on")
        if not result.success:
            self.logger.error("Unable to power on robot message was " + result.message)
            return False
        return True

    def move(self) -> None:
        self.logger.info("Standing robot up")
        result = self.robot.command("stand")
        if not result.success:
            self.logger.error("Robot did not stand message was " + result.message)
            return
        self.logger.info("Successfully stood up.")
        time.sleep(3)

        # Move the arm to a spot in front of the robot, and command the body to follow the hand.
        # Build a position to move the arm to (in meters, relative to the body frame origin.)
        x = 1.25
        y = 0
        z = 0.25
        hand_pos_rt_body = geometry_pb2.Vec3(x=x, y=y, z=z)

        # Rotation as a quaternion.
        qw = 1
        qx = 0
        qy = 0
        qz = 0
        body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Build the SE(3) pose of the desired hand position in the moving body frame.
        body_T_hand = geometry_pb2.SE3Pose(position=hand_pos_rt_body, rotation=body_Q_hand)

        # Transform the desired from the moving body frame to the odom frame.
        odom_T_body = self.tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        odom_T_body_se3 = math_helpers.SE3Pose(
            odom_T_body.transform.translation.x,
            odom_T_body.transform.translation.y,
            odom_T_body.transform.translation.z,
            math_helpers.Quat(
                odom_T_body.transform.rotation.w,
                odom_T_body.transform.rotation.x,
                odom_T_body.transform.rotation.y,
                odom_T_body.transform.rotation.z,
            ),
        )

        odom_T_hand = odom_T_body_se3 * math_helpers.SE3Pose.from_proto(body_T_hand)

        # duration in seconds
        seconds = 5

        # Create the arm command.
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x,
            odom_T_hand.y,
            odom_T_hand.z,
            odom_T_hand.rot.w,
            odom_T_hand.rot.x,
            odom_T_hand.rot.y,
            odom_T_hand.rot.z,
            ODOM_FRAME_NAME,
            seconds,
        )

        # Tell the robot's body to follow the arm
        follow_arm_command = RobotCommandBuilder.follow_arm_command()

        # Combine the arm and mobility commands into one synchronized command.
        cmd = RobotCommandBuilder.build_synchro_command(follow_arm_command, arm_command)

        # Convert to ROS2 action
        action_goal = RobotCommand.Goal()
        convert(cmd, action_goal.command)
        self.logger.info("Performing arm with body follow")

        # Send the command
        self.robot_command_client.send_goal_and_wait("arm_with_body_follow", action_goal)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_spot = ArmWithBodyFollow(args.robot)
    hello_spot.initialize_robot()
    hello_spot.move()


if __name__ == "__main__":
    main()
