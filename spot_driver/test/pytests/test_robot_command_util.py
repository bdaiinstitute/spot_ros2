# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Tests to check trajectory batching in robot commands.
"""

import math
import time
from typing import Callable, Optional, Tuple

import pytest
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

from spot_driver.robot_command_util import batch_command, get_batch_size, min_time_since_reference

ContinuousTrajectory1D = Callable[[float], float]
ContinuousTrajectory2D = Callable[[float], Tuple[SE2Pose, SE2Velocity]]
ContinuousTrajectory3D = Callable[[float], Tuple[SE3Pose, SE3Velocity]]


def gripper_continuous_trajectory(t: float) -> float:
    """
    We use this function to model a continuous gripper trajectory over time.
    """
    radius = 1.0
    period = 2.0
    x = radius * math.cos(2 * math.pi * t / period)
    return x


def mobility_continuous_trajectory(t: float) -> Tuple[SE2Pose, SE2Velocity]:
    """
    we use this function to model a continuous mobility trajectory over time.
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


def arm_continuous_trajectory(t: float) -> Tuple[SE3Pose, SE3Velocity]:
    """
    we use this function to model a continuous arm trajectory over time.
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


def gripper_discrete_trajectory(
    reference_time: float, ramp_up_time: float, duration: float, dt: float, trajectory_function: ContinuousTrajectory1D
) -> trajectory_pb2.ScalarTrajectory:
    """
    We use this function to model a discrete gripper trajectory over time.
    """
    trajectory = trajectory_pb2.ScalarTrajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
    trajectory.interpolation = trajectory_pb2.POS_INTERP_CUBIC
    t = 0.0
    while t < duration:
        pos = trajectory_function(t)
        point = trajectory.points.add()
        point.point = pos
        point.time_since_reference.CopyFrom(seconds_to_duration(t + ramp_up_time))
        t = t + dt
    return trajectory


def mobility_discrete_trajectory(
    reference_time: float, ramp_up_time: float, duration: float, dt: float, trajectory_function: ContinuousTrajectory2D
) -> trajectory_pb2.SE2Trajectory:
    """
    We use this function to model a discrete mobility trajectory over time.
    """
    trajectory = trajectory_pb2.SE2Trajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
    trajectory.interpolation = trajectory_pb2.POS_INTERP_CUBIC
    t = 0.0
    while t < duration:
        pos, _ = trajectory_function(t)
        point = trajectory.points.add()
        point.pose.CopyFrom(pos.to_proto())
        point.time_since_reference.CopyFrom(seconds_to_duration(t + ramp_up_time))
        t = t + dt
    return trajectory


def arm_discrete_trajectory(
    reference_time: float, ramp_up_time: float, duration: float, dt: float, trajectory_function: ContinuousTrajectory3D
) -> trajectory_pb2.SE3Trajectory:
    """
    We use this function to model a discrete arm trajectory over time.
    """
    trajectory = trajectory_pb2.SE3Trajectory()
    trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
    trajectory.pos_interpolation = trajectory_pb2.POS_INTERP_CUBIC
    trajectory.ang_interpolation = trajectory_pb2.ANG_INTERP_CUBIC_EULER
    t = 0.0
    while t < duration:
        pos, vel = trajectory_function(t)
        point = trajectory.points.add()
        point.pose.CopyFrom(pos.to_proto())
        point.velocity.CopyFrom(vel.to_proto())
        point.time_since_reference.CopyFrom(seconds_to_duration(t + ramp_up_time))
        t = t + dt
    return trajectory


def build_test_command(
    hand_trajectory: Optional[trajectory_pb2.SE3Trajectory] = None,
    mobility_trajectory: Optional[trajectory_pb2.SE2Trajectory] = None,
    gripper_trajectory: Optional[trajectory_pb2.ScalarTrajectory] = None,
) -> robot_command_pb2.RobotCommand:
    """
    Return a robot command with three optional trajectories.

    Args
        hand_trajectory: A 3D hand trajectory.
        mobility_trajectory: A 2D mobility trajectory.
        gripper_trajectory: A scalar gripper trajectory.

    Returns:
        A robot command with any of the given optional trajectories.
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
        arm_command=arm_request, mobility_command=mobility_request, gripper_command=gripper_request
    )
    command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
    return command


def test_get_batch_size() -> None:
    """
    Test the command get_batch_size.
    """
    sequence = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]
    batch_size = 5
    overlapping = 0

    assert get_batch_size(len(sequence), batch_size, overlapping, 0) == 5
    assert get_batch_size(len(sequence), batch_size, overlapping, 1) == 5
    assert get_batch_size(len(sequence), batch_size, overlapping, 2) == 0

    sequence = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C"]
    batch_size = 5
    overlapping = 1

    assert get_batch_size(len(sequence), batch_size, overlapping, 0) == 5
    assert get_batch_size(len(sequence), batch_size, overlapping, 1) == 5
    assert get_batch_size(len(sequence), batch_size, overlapping, 2) == 5
    assert get_batch_size(len(sequence), batch_size, overlapping, 3) == 1
    assert get_batch_size(len(sequence), batch_size, overlapping, 4) == 0


def test_trajectories_different_length() -> None:
    """
    When a command contains more than one trajectory longer than the batch size,
    with different sizes, we cannot batch.
    """

    hand_trajectory: trajectory_pb2.SE3Trajectory = arm_discrete_trajectory(
        reference_time=time.time(), ramp_up_time=0, duration=5, dt=0.1, trajectory_function=arm_continuous_trajectory
    )
    mobility_trajectory: trajectory_pb2.SE2Trajectory = mobility_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=0,
        duration=10,
        dt=0.1,
        trajectory_function=mobility_continuous_trajectory,
    )

    command = build_test_command(hand_trajectory=hand_trajectory, mobility_trajectory=mobility_trajectory)
    commands = batch_command(command=command, batch_size=50)

    # Trajectories have different lengths so we expect one command.
    assert len(commands) == 1


def test_trajectories_not_aligned() -> None:
    """
    When a command contains more than one trajectory longer than the batch size,
    not time aligned, we cannot batch.
    """

    hand_trajectory: trajectory_pb2.SE3Trajectory = arm_discrete_trajectory(
        reference_time=time.time(), ramp_up_time=0, duration=5, dt=0.1, trajectory_function=arm_continuous_trajectory
    )
    mobility_trajectory: trajectory_pb2.SE2Trajectory = mobility_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=0,
        duration=10,
        dt=0.2,
        trajectory_function=mobility_continuous_trajectory,
    )

    command = build_test_command(hand_trajectory=hand_trajectory, mobility_trajectory=mobility_trajectory)
    commands = batch_command(command=command, batch_size=10)

    # Trajectories are not aligned so we expect one command.
    assert len(commands) == 1


def test_multiple_trajectories() -> None:
    """
    When a command contains more than one trajectory longer than the batch size,
    and time aligned, we can batch.
    """

    hand_trajectory: trajectory_pb2.SE3Trajectory = arm_discrete_trajectory(
        reference_time=time.time(), ramp_up_time=0, duration=20, dt=0.1, trajectory_function=arm_continuous_trajectory
    )
    mobility_trajectory: trajectory_pb2.SE2Trajectory = mobility_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=0,
        duration=20,
        dt=0.1,
        trajectory_function=mobility_continuous_trajectory,
    )
    gripper_trajectory: trajectory_pb2.ScalarTrajectory = gripper_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=0,
        duration=20,
        dt=0.1,
        trajectory_function=gripper_continuous_trajectory,
    )

    command = build_test_command(
        hand_trajectory=hand_trajectory,
        mobility_trajectory=mobility_trajectory,
        gripper_trajectory=gripper_trajectory,
    )
    commands = batch_command(command=command, batch_size=50, overlapping=0)

    # Each trajectory contains 200 = 20 / 0.1 datapoints.
    # With a batch size of 50, we expect 4 commands.
    assert len(commands) == 4

    assert len(commands[0].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 50
    assert len(commands[1].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 50
    assert len(commands[2].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 50
    assert len(commands[3].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 50

    assert len(commands[0].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 50
    assert len(commands[1].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 50
    assert len(commands[2].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 50
    assert len(commands[3].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 50

    assert len(commands[0].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 50
    assert len(commands[1].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 50
    assert len(commands[2].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 50
    assert len(commands[3].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 50


def test_multiple_trajectorties_with_stride() -> None:
    """
    When a command contains more than one trajectory longer than the batch size,
    and time aligned, we can batch.
    """

    hand_trajectory: trajectory_pb2.SE3Trajectory = arm_discrete_trajectory(
        reference_time=time.time(), ramp_up_time=0, duration=5, dt=0.1, trajectory_function=arm_continuous_trajectory
    )
    mobility_trajectory: trajectory_pb2.SE2Trajectory = mobility_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=0,
        duration=5,
        dt=0.1,
        trajectory_function=mobility_continuous_trajectory,
    )
    gripper_trajectory: trajectory_pb2.ScalarTrajectory = gripper_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=0,
        duration=5,
        dt=0.1,
        trajectory_function=gripper_continuous_trajectory,
    )

    command = build_test_command(
        hand_trajectory=hand_trajectory,
        mobility_trajectory=mobility_trajectory,
        gripper_trajectory=gripper_trajectory,
    )
    commands = batch_command(command=command, batch_size=20, overlapping=4)

    # Each trajectory contains 51 = 1 + 5 / 0.1 datapoints.
    # With a batch size of 20, and 4 overlapping points between trajectories,
    # we expect 3 commands.
    assert len(commands) == 3

    assert len(commands[0].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 20
    assert len(commands[1].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 20
    assert len(commands[2].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 19

    assert len(commands[0].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 20
    assert len(commands[1].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 20
    assert len(commands[2].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 19

    assert len(commands[0].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 20
    assert len(commands[1].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 20
    assert len(commands[2].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 19


def test_one_trajectory_with_stride() -> None:
    """
    When a command contains only one trajectory longer than the batch size,
    we can always batch.
    """

    hand_trajectory: trajectory_pb2.SE3Trajectory = arm_discrete_trajectory(
        reference_time=time.time(), ramp_up_time=0, duration=5, dt=0.1, trajectory_function=arm_continuous_trajectory
    )

    command = build_test_command(hand_trajectory=hand_trajectory)
    commands = batch_command(command=command, batch_size=20, overlapping=4)

    # One trajectory contains 51 = 1 + 5 / 0.1 datapoints.
    # With a batch size of 20, and 4 overlapping points between trajectories,
    # we expect 3 commands.
    assert len(commands) == 3

    assert len(commands[0].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 20
    assert len(commands[1].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 20
    assert len(commands[2].synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points) == 19

    assert len(commands[0].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 0
    assert len(commands[1].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 0
    assert len(commands[2].synchronized_command.mobility_command.se2_trajectory_request.trajectory.points) == 0

    assert len(commands[0].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 0
    assert len(commands[1].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 0
    assert len(commands[2].synchronized_command.gripper_command.claw_gripper_command.trajectory.points) == 0


def test_batch_execution_time() -> None:
    """
    Check the execution time of each batch.

    Test overview:

        trajectory
        pos:   0    1     2     3     4     5     6     7     8     9    10
        time:  4.0  4.1   4.2   4.3   4.4   4.5   4.6   4.7   4.8   4.9  5.0

        batch 0
        pos:   0    1     2     3     4
        time:  4.0  4.1   4.2   4.3   4.4
        execution_time: 4

        batch 1
        pos:                    3     4     5     6     7
        time:                   4.3   4.4   4.5   4.6   4.7
        execution_time: 4 + (5 - 2) * 0.1

        batch 2
        pos:                                      6     7     8     9    10
        time:                                     4.6   4.7   4.8   4.9  5.0
        execution_time: 4 + 2 * (5 - 2) * 0.1

    """
    ramp_up_time = 4
    duration = 1
    batch_size = 5
    time_sample = 0.1
    overlapping = 2

    hand_trajectory: trajectory_pb2.SE3Trajectory = arm_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=ramp_up_time,
        duration=duration,
        dt=time_sample,
        trajectory_function=arm_continuous_trajectory,
    )
    mobility_trajectory: trajectory_pb2.SE2Trajectory = mobility_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=ramp_up_time,
        duration=duration,
        dt=time_sample,
        trajectory_function=mobility_continuous_trajectory,
    )
    gripper_trajectory: trajectory_pb2.ScalarTrajectory = gripper_discrete_trajectory(
        reference_time=time.time(),
        ramp_up_time=ramp_up_time,
        duration=duration,
        dt=time_sample,
        trajectory_function=gripper_continuous_trajectory,
    )

    command = build_test_command(
        hand_trajectory=hand_trajectory,
        mobility_trajectory=mobility_trajectory,
        gripper_trajectory=gripper_trajectory,
    )
    commands = batch_command(command=command, batch_size=batch_size, overlapping=overlapping)

    assert len(commands) == 3

    sequence_length = len(hand_trajectory.points)
    assert get_batch_size(sequence_length, batch_size, overlapping, 0) == 5
    assert get_batch_size(sequence_length, batch_size, overlapping, 1) == 5
    assert get_batch_size(sequence_length, batch_size, overlapping, 2) == 5

    assert min_time_since_reference(commands[0]) == pytest.approx(ramp_up_time)
    assert min_time_since_reference(commands[1]) == pytest.approx(
        ramp_up_time + (batch_size - overlapping) * time_sample
    )
    assert min_time_since_reference(commands[2]) == pytest.approx(
        ramp_up_time + 2 * (batch_size - overlapping) * time_sample
    )
