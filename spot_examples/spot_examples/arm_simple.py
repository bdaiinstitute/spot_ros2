import argparse
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

import spot_driver.conversions as conv
from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander
from .tf_listener_wrapper import TFListenerWrapper


def hello_arm(robot_name: Optional[str] = None) -> bool:
    # Set up basic ROS2 utilities for communicating with the driver
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    logger = node.get_logger()

    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    robot = SimpleSpotCommander(robot_name, node)
    robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)

    # Claim robot
    logger.info("Claiming robot")
    result = robot.command("claim")
    if not result.success:
        node.get_logger().error("Unable to claim robot message was " + result.message)
        return False
    logger.info("Claimed robot")

    # Stand the robot up.
    logger.info("Powering robot on")
    result = robot.command("power_on")
    if not result.success:
        logger.error("Unable to power on robot message was " + result.message)
        return False
    logger.info("Standing robot up")
    result = robot.command("stand")
    if not result.success:
        logger.error("Robot did not stand message was " + result.message)
        return False
    logger.info("Successfully stood up.")

    # Move the arm to a spot in front of the robot, and open the gripper.

    # Make the arm pose RobotCommand
    # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).
    x = 0.75
    y = 0
    z = 0.25
    hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

    # Rotation as a quaternion
    qw = 1
    qx = 0
    qy = 0
    qz = 0
    flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

    flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

    odom_T_flat_body = tf_listener.lookup_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)

    # duration in seconds
    seconds = 2

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

    # Make the open gripper RobotCommand
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

    # Combine the arm and gripper commands into one RobotCommand
    command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    conv.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    logger.info("Moving arm to position 1.")
    robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

    # Move the arm to a different position
    hand_ewrt_flat_body.z = 0

    flat_body_Q_hand.w = 0.707
    flat_body_Q_hand.x = 0.707
    flat_body_Q_hand.y = 0
    flat_body_Q_hand.z = 0

    flat_body_T_hand2 = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)
    odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand2)

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

    # Close the gripper
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)

    # Build the proto
    command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    conv.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    logger.info("Moving arm to position 2.")
    robot_command_client.send_goal_and_wait("arm_move_two", action_goal)

    return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_arm(args.robot)


if __name__ == "__main__":
    main()
