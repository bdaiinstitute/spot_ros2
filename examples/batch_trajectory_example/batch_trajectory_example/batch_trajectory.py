#!/usr/bin/python3

# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member


"""
This example shows how a long trajectory is sent to the robot in batches.
"""

import argparse
import math
import time
from typing import Callable, Optional, Tuple

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import (
    arm_command_pb2,
    basic_command_pb2,
    gripper_command_pb2,
    mobility_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
    trajectory_pb2,
)
from bosdyn.client.frame_helpers import (
    ODOM_FRAME_NAME,
)
from bosdyn.client.math_helpers import Quat, SE3Pose, SE3Velocity
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration, seconds_to_timestamp
from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from utilities.simple_spot_commander import SimpleSpotCommander

from spot_msgs.action import RobotCommand  # type: ignore

###############################################################################
# CONTINUOUS TRAJECTORIES
# Following, some continuous functions to generate sample trajectories for our
# tests.

ContinuousTrajectory3D = Callable[[float], Tuple[SE3Pose, SE3Velocity]]


def _continuous_trajectory_3d(t: float) -> Tuple[SE3Pose, SE3Velocity]:
    """
    Given a time t in the trajectory, return the SE3Pose and SE3Velocity at
    this point in the trajectory.
    """

    # Draw a circle
    radius = 0.25  # Circle radius in meters
    period = 2.0  # Time required to go all the way around circle in seconds.
    x = radius * math.cos(2 * math.pi * t / period)
    y = radius * math.sin(2 * math.pi * t / period)
    z = 0.0
    quat = Quat(1, 0, 0, 0)
    vx = -radius * 2 * math.pi / period * math.sin(2 * math.pi * t / period)
    vy = radius * 2 * math.pi / period * math.cos(2 * math.pi * t / period)
    vz = 0.0
    return SE3Pose(x, y, z, quat), SE3Velocity(vx, vy, vz, 0, 0, 0)


###############################################################################
# DISCRETE TRAJECTORIES
# Following, some trajectories created by sampling previously defined
# continuous functions.


def _discrete_trajectory_3d(
    duration: float, dt: float, trajectory_function: ContinuousTrajectory3D
) -> trajectory_pb2.SE3Trajectory:
    """
    Return a trajectory in 3D space.
    """
    start_time = time.time()
    trajectory = trajectory_pb2.SE3Trajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(start_time))
    trajectory.pos_interpolation = trajectory_pb2.POS_INTERP_CUBIC
    trajectory.ang_interpolation = trajectory_pb2.ANG_INTERP_CUBIC_EULER
    t = start_time
    while t - start_time < duration:
        pos, vel = trajectory_function(t)
        point = trajectory.points.add()
        point.pose.CopyFrom(pos.to_proto())
        point.velocity.CopyFrom(vel.to_proto())
        point.time_since_reference.CopyFrom(seconds_to_duration(t - start_time))
        t = t + dt
    return trajectory


###############################################################################
# ROBOT COMMAND
# Here we build a robot command containing different trajectories generated
# from the previously defined samples.


def _build_sample_command(
    robot_name: str,
    hand_trajectory: Optional[trajectory_pb2.SE3Trajectory] = None,
    mobility_trajectory: Optional[trajectory_pb2.SE2Trajectory] = None,
    gripper_trajectory: Optional[trajectory_pb2.ScalarTrajectory] = None,
) -> robot_command_pb2.RobotCommand:
    """
    Return a robot command with three optional trajectories.

    Args
        hand_trajectory: An hand 3D trajectory.
        mobility_trajectory: A mobility 2D rajectory.
        gripper_trajectory: A gripper scalar trajectory.

    Returns:
        A robot command with any of the given optional trajectory.
    """

    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)

    # Build an Arm request.

    arm_request = None
    if hand_trajectory is not None:
        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            root_frame_name=odom_frame_name,
            pose_trajectory_in_task=hand_trajectory,
        )
        arm_request = arm_command_pb2.ArmCommand.Request(arm_cartesian_command=arm_cartesian_command)

    # Build a Mobility request.

    mobility_request = None
    if mobility_trajectory is not None:
        mobility_request = basic_command_pb2.SE2TrajectoryCommand.Request(trajectory=mobility_trajectory)
        mobility_request = mobility_command_pb2.MobilityCommand.Request(se2_trajectory_request=mobility_request)

    # Build a Gripper request.

    gripper_request = None
    if gripper_trajectory is not None:
        claw_gripper_request = gripper_command_pb2.ClawGripperCommand.Request(trajectory=gripper_trajectory)
        gripper_request = gripper_command_pb2.GripperCommand.Request(claw_gripper_command=claw_gripper_request)

    # Build a Robot Command.

    synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
        arm_command=arm_request, mobility_command=mobility_request, gripper_command=gripper_request
    )
    command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
    return command


###############################################################################
# TEST RUNNER
# This class claims the robot, set it up for the test and executes the test.


class SpotRunner:
    """
    This example show how send a long trajectory.
    """

    def __init__(self, node: Node, args: argparse.Namespace):
        self._node = node
        self._robot_name: str = args.robot
        self._logger = node.get_logger()
        self._robot = SimpleSpotCommander(self._robot_name)

        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )

    def _ready_arm(self) -> bool:
        """
        Unstow the robot arm.
        """
        command = RobotCommandBuilder.arm_ready_command()
        action_goal = RobotCommand.Goal()
        convert.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("ready_arm", action_goal)

    def _arm_stow(self) -> bool:
        """
        Stow the robot arm.
        """
        command = RobotCommandBuilder.arm_stow_command()
        action_goal = RobotCommand.Goal()
        convert.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("arm_stow", action_goal)

    def test_run(self) -> bool:
        """
        Send a very long arm trajectory to Spot.
        Returns:
            True the process runs without errors, False otherwise.
        """

        # Claim robot.
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result:
            self._logger.error("Unable to claim robot")
            return False
        self._logger.info("Claimed robot")

        # Power on robot.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result:
            self._logger.error("Unable to power on robot")
            return False

        # Stand up robot.
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result:
            self._logger.error("Robot did not stand")
            return False
        self._logger.info("Successfully stood up.")

        # Unstow the arm.
        self._logger.info("Unstow the arm")
        result = self._ready_arm()
        if not result:
            self._logger.error("Failed to unstow the arm.")
            return False
        self._logger.info("Arm ready.")

        # Arm command.

        hand_trajectory: trajectory_pb2.SE3Trajectory = _discrete_trajectory_3d(
            duration=5, dt=0.1, trajectory_function=_continuous_trajectory_3d
        )
        command = _build_sample_command(robot_name=self._robot_name, hand_trajectory=hand_trajectory)
        action_goal = RobotCommand.Goal()
        convert.convert_proto_to_bosdyn_msgs_robot_command(command, action_goal.command)
        self._robot_command_client.send_goal_and_wait("arm_move_one", goal=action_goal, timeout_sec=5)

        # Stow the arm.
        self._logger.info("Stow the arm")
        result = self._arm_stow()
        if not result:
            self._logger.error("Failed to stow the arm.")
            return False
        self._logger.info("Arm stowed.")

        return True


def cli() -> argparse.ArgumentParser:
    """
    Parse all arguments.
    --robot [string]
        The robot name e.g. Opal.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="The robot name.")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    """
    Execute the example.
    """
    # Set up basic ROS2 utilities for communicating with the driver.
    node = ros_scope.node()
    if node is None:
        raise ValueError("No ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")

    spot_runner = SpotRunner(node, args)
    spot_runner.test_run()


if __name__ == "__main__":
    main()
