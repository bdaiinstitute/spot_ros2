# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Tests to check trajectory batching in robot commands.
"""

# pylint: disable=E1101

import math
import time
from typing import Callable, Optional, Tuple

from bosdyn.api import (
    arm_command_pb2,
    basic_command_pb2,
    gripper_command_pb2,
    mobility_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
    trajectory_pb2,
)
from bosdyn.client.math_helpers import Quat, SE2Pose, SE2Velocity, SE3Pose, SE3Velocity
from bosdyn.util import seconds_to_duration, seconds_to_timestamp

from spot_driver.robot_command_util import batch_command

###############################################################################
# CONTINUOUS TRAJECTORIES
# Following, some continuous functions to generate sample trajectories for our
# tests.

ContinuousTrajectory1D = Callable[[float], float]
ContinuousTrajectory2D = Callable[[float], Tuple[SE2Pose, SE2Velocity]]
ContinuousTrajectory3D = Callable[[float], Tuple[SE3Pose, SE3Velocity]]


def _continuous_trajectory_1d(t: float) -> float:
    """
    Given a time t in the trajectory, return a scalar representing a value
    in the trajectory.
    """
    radius = 1.0
    period = 2.0
    x = radius * math.cos(2 * math.pi * t / period)
    return x


def _continuous_trajectory_2d(t: float) -> Tuple[SE2Pose, SE2Velocity]:
    """
    Given a time t in the trajectory, return the SE3Pose and SE3Velocity at
    this point in the trajectory.
    """

    # Draw a circle
    radius = 0.25  # Circle radius in meters
    period = 2.0  # Time required to go all the way around circle in seconds.
    x = radius * math.cos(2 * math.pi * t / period)
    y = radius * math.sin(2 * math.pi * t / period)
    angle = 0.0
    vx = -radius * 2 * math.pi / period * math.sin(2 * math.pi * t / period)
    vy = radius * 2 * math.pi / period * math.cos(2 * math.pi * t / period)
    va = 0
    return SE2Pose(x, y, angle), SE2Velocity(vx, vy, va)


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


def _discrete_trajectory_1d(
    duration: float, dt: float, trajectory_function: ContinuousTrajectory1D
) -> trajectory_pb2.ScalarTrajectory:
    """
    Return a trajectory value.
    """
    start_time = time.time()
    trajectory = trajectory_pb2.ScalarTrajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(start_time))
    trajectory.interpolation = trajectory_pb2.POS_INTERP_CUBIC
    t = start_time
    while t - start_time < duration:
        pos = trajectory_function(t)
        point = trajectory.points.add()
        point.point = pos
        point.time_since_reference.CopyFrom(seconds_to_duration(t - start_time))
        t = t + dt
    return trajectory


def _discrete_trajectory_2d(
    duration: float, dt: float, trajectory_function: ContinuousTrajectory2D
) -> trajectory_pb2.SE2Trajectory:
    """
    Return a trajectory in 2D space.
    """
    start_time = time.time()
    trajectory = trajectory_pb2.SE2Trajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(start_time))
    trajectory.interpolation = trajectory_pb2.POS_INTERP_CUBIC
    t = start_time
    while t - start_time < duration:
        pos, _ = trajectory_function(t)
        point = trajectory.points.add()
        point.pose.CopyFrom(pos.to_proto())
        point.time_since_reference.CopyFrom(seconds_to_duration(t - start_time))
        t = t + dt
    return trajectory


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
        frame_name = "body"
        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            root_frame_name=frame_name, pose_trajectory_in_task=hand_trajectory
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
        mobility_command=mobility_request, arm_command=arm_request, gripper_command=gripper_request
    )
    command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
    return command


def test_trajectory_batching() -> None:
    """
    Test if a command containing trajectory longer than a given batch size is
    correctly batched.
    """

    hand_trajectory: trajectory_pb2.SE3Trajectory = _discrete_trajectory_3d(
        duration=20, dt=0.1, trajectory_function=_continuous_trajectory_3d
    )
    mobility_trajectory: trajectory_pb2.SE2Trajectory = _discrete_trajectory_2d(
        duration=20, dt=0.1, trajectory_function=_continuous_trajectory_2d
    )
    gripper_trajectory: trajectory_pb2.ScalarTrajectory = _discrete_trajectory_1d(
        duration=20, dt=0.1, trajectory_function=_continuous_trajectory_1d
    )

    batch_size = 50
    command = _build_sample_command(
        hand_trajectory=hand_trajectory,
        mobility_trajectory=mobility_trajectory,
        gripper_trajectory=gripper_trajectory,
    )
    commands = batch_command(command, batch_size)
    print(len(commands))
