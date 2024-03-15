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
from typing import Callable, Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import (
    arm_command_pb2,
    basic_command_pb2,
    geometry_pb2,
    gripper_command_pb2,
    mobility_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
    trajectory_pb2,
)
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration, seconds_to_timestamp
from bosdyn_msgs.conversions import convert
from google.protobuf.wrappers_pb2 import DoubleValue
from rclpy.node import Node

from spot_examples.simple_spot_commander import SimpleSpotCommander
from spot_msgs.action import RobotCommand  # type: ignore
from tf2_ros import TransformStamped

###############################################################################
# CONTINUOUS TRAJECTORIES
# Following, some continuous functions to generate sample trajectories for our
# tests.

ContinuousTrajectory3D = Callable[[float], SE3Pose]


def _continuous_trajectory_3d(t: float) -> SE3Pose:
    """
    Given a time t in the trajectory, return the SE3Pose at this point in
    the trajectory.
    """

    # Draw a Rhodonea curve with n petals and period P (seconds)
    n = 3
    P = 10.0
    t_norm = t / P
    radius = 0.4 * math.sin(math.pi * n * t_norm)
    x = radius * math.cos(math.pi * t_norm)
    y = radius * math.sin(math.pi * t_norm)
    z = 0.0
    quat = Quat(1, 0, 0, 0)
    return SE3Pose(x, y, z, quat)


###############################################################################
# DISCRETE TRAJECTORIES
# Following, some trajectories created by sampling previously defined
# continuous functions.


def _discrete_trajectory_3d(
    reference_time: float,
    start_time: float,
    duration: float,
    dt: float,
    trajectory_function: ContinuousTrajectory3D,
) -> trajectory_pb2.SE3Trajectory:
    """
    Return a discrete trajectory in 3D space by sampling a continuous 3D function.
    The continuous function is sampled from a given start time, for a specified
    duration and at given sampling intervals.


    Args:
        reference_time: The time the trajectory is executed.
        start_time: The initial sampling time.
        duration: The total sampling time.
        dt: The sampling interval.
        trajectory_function: the trajectory function to sample.
    """
    trajectory = trajectory_pb2.SE3Trajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
    t = start_time
    while t - start_time <= duration:
        pos = trajectory_function(t)
        point = trajectory.points.add()
        point.pose.CopyFrom(pos.to_proto())
        point.time_since_reference.CopyFrom(seconds_to_duration(t - start_time))
        t = t + dt
    return trajectory


###############################################################################
# ROBOT COMMAND
# Here we build a robot command containing different trajectories generated
# from the previously defined samples.


def _build_sample_command(
    root_frame_name: str,
    wrist_tform_tool: geometry_pb2.SE3Pose,
    root_tform_task: geometry_pb2.SE3Pose,
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

    # Build an Arm request.

    arm_request = None
    if hand_trajectory is not None:
        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            root_frame_name=root_frame_name,
            pose_trajectory_in_task=hand_trajectory,
            wrist_tform_tool=wrist_tform_tool,
            root_tform_task=root_tform_task,
            maximum_acceleration=DoubleValue(value=10000.0),
            max_linear_velocity=DoubleValue(value=10000.0),
            max_angular_velocity=DoubleValue(value=10000.0),
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

    def _to_se3(self, ros_transform: TransformStamped) -> SE3Pose:
        """
        Convert from ROS TransformStamped to Bosdyn SE3Pose
        """
        return SE3Pose(
            ros_transform.transform.translation.x,
            ros_transform.transform.translation.y,
            ros_transform.transform.translation.z,
            Quat(
                ros_transform.transform.rotation.w,
                ros_transform.transform.rotation.x,
                ros_transform.transform.rotation.y,
                ros_transform.transform.rotation.z,
            ),
        )

    def _ready_arm(self) -> bool:
        """
        Unstow the robot arm.
        """
        command = RobotCommandBuilder.arm_ready_command()
        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("ready_arm", action_goal)

    def _arm_stow(self) -> bool:
        """
        Stow the robot arm.
        """
        command = RobotCommandBuilder.arm_stow_command()
        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("arm_stow", action_goal)

    def _follow_trajectory(
        self,
        root_frame_name: str,
        wrist_tform_tool: geometry_pb2.SE3Pose,
        root_tform_task: geometry_pb2.SE3Pose,
        hand_trajectory: Optional[trajectory_pb2.SE3Trajectory] = None,
        mobility_trajectory: Optional[trajectory_pb2.SE2Trajectory] = None,
        gripper_trajectory: Optional[trajectory_pb2.ScalarTrajectory] = None,
    ) -> None:
        """
        Order the arm tool to follow a given trajectory.
        """
        command = _build_sample_command(
            root_frame_name=root_frame_name,
            root_tform_task=root_tform_task,
            wrist_tform_tool=wrist_tform_tool,
            hand_trajectory=hand_trajectory,
            mobility_trajectory=mobility_trajectory,
            gripper_trajectory=gripper_trajectory,
        )
        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)
        self._robot_command_client.send_goal_and_wait("move_arm", goal=action_goal)

    def test_run(self) -> bool:
        """
        Send a very long arm trajectory to Spot.
        Returns:
            True the process runs without errors, False otherwise.
        """

        odom_frame_name = namespace_with(self._robot_name, ODOM_FRAME_NAME)
        grav_body_frame_name = namespace_with(self._robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        tf_listener = TFListenerWrapper(self._node)
        tf_listener.wait_for_a_tform_b(odom_frame_name, grav_body_frame_name)

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

        # Create a task frame. This will be the frame the trajectory is defined relative to.
        # In this example, that is the center of the circle.
        # The frame at the center of the circle is one that is 90cm in front of the robot,
        # with z pointing back at the robot, x off the right side of the robot, and y up
        grav_body_T_task = SE3Pose(x=0.9, y=0, z=0, rot=Quat(w=0.5, x=0.5, y=-0.5, z=-0.5))

        # Now, get the transform between the "odometry" frame and the gravity aligned body frame.
        # This will be used in conjunction with the grav_body_T_task frame to get the
        # transformation between the odometry frame and the task frame. In order to get
        # odom_T_grav_body we use a snapshot of the frame tree. For more information on the frame
        # tree, see https://dev.bostondynamics.com/docs/concepts/geometry_and_frames
        odom_T_grav_body: SE3Pose = self._to_se3(tf_listener.lookup_a_tform_b(odom_frame_name, grav_body_frame_name))

        odom_T_task: SE3Pose = odom_T_grav_body * grav_body_T_task
        wrist_tform_tool = SE3Pose(x=0.25, y=0, z=0, rot=Quat(w=0.5, x=0.5, y=-0.5, z=-0.5))

        # Move to the first position of the sampled trajectory.
        hand_trajectory: trajectory_pb2.SE3Trajectory = _discrete_trajectory_3d(
            reference_time=time.time() + 2,
            start_time=0,
            duration=0,
            dt=0.1,
            trajectory_function=_continuous_trajectory_3d,
        )
        self._follow_trajectory(
            root_frame_name=ODOM_FRAME_NAME,
            root_tform_task=odom_T_task.to_proto(),
            wrist_tform_tool=wrist_tform_tool.to_proto(),
            hand_trajectory=hand_trajectory,
        )

        # Execute the sampled trajectory.
        hand_trajectory: trajectory_pb2.SE3Trajectory = _discrete_trajectory_3d(
            reference_time=time.time() + 1,
            start_time=0,
            duration=10,
            dt=0.05,
            trajectory_function=_continuous_trajectory_3d,
        )
        self._follow_trajectory(
            root_frame_name=ODOM_FRAME_NAME,
            root_tform_task=odom_T_task.to_proto(),
            wrist_tform_tool=wrist_tform_tool.to_proto(),
            hand_trajectory=hand_trajectory,
        )

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
