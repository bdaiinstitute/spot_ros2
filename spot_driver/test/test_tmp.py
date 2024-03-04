"""
Working with trajectories.
"""

# pylint: disable=E1101

import logging
import math
import time
from typing import Callable, Tuple

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

ContinuousTrajectory1D = Callable[[float], float]
ContinuousTrajectory2D = Callable[[float], Tuple[SE2Pose, SE2Velocity]]
ContinuousTrajectory3D = Callable[[float], Tuple[SE3Pose, SE3Velocity]]

logging.basicConfig(level=logging.INFO)

# CONTINOUS TRAJECTORIES
# Here we define some continuous functions to use generate trajectories.


def continuous_trajectory_1d(t: float) -> float:
    """
    Given a time t in the trajectory, return a scalar representing a value
    in the trajectory.
    """
    radius = 1.0
    period = 2.0
    x = radius * math.cos(2 * math.pi * t / period)
    return x


def continuous_trajectory_2d(t: float) -> Tuple[SE2Pose, SE2Velocity]:
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


def continuous_trajectory_3d(t: float) -> Tuple[SE3Pose, SE3Velocity]:
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


# DISCRETE TRAJECTORIES
# Here we sample the continuous functions to create discrete trajectories.


def dicrete_trajectory_1d(
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


def dicrete_trajectory_2d(
    duration: float, dt: float, trajectory_function: ContinuousTrajectory2D
) -> trajectory_pb2.SE2Trajectory:
    """
    Return a trajectory in 3D space.
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


def dicrete_trajectory_3d(
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


def build_robot_command() -> robot_command_pb2.RobotCommand:
    """
    Return a robot command with three components:
    1 - an Arm command;
    2 - a Mobility command;
    3 - a Gripper command.
    """

    # Build an Arm request.

    frame_name = "body"
    hand_trajectory: trajectory_pb2.SE3Trajectory = dicrete_trajectory_3d(
        duration=20, dt=0.1, trajectory_function=continuous_trajectory_3d
    )
    arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
        root_frame_name=frame_name, pose_trajectory_in_task=hand_trajectory
    )
    arm_request = arm_command_pb2.ArmCommand.Request(arm_cartesian_command=arm_cartesian_command)

    # Build a Mobility request.

    mobility_trajectory: trajectory_pb2.SE2Trajectory = dicrete_trajectory_2d(
        duration=20, dt=0.1, trajectory_function=continuous_trajectory_2d
    )
    mobility_request = basic_command_pb2.SE2TrajectoryCommand.Request(trajectory=mobility_trajectory)
    mobility_request = mobility_command_pb2.MobilityCommand.Request(se2_trajectory_request=mobility_request)

    # Build a Gripper request.

    gripper_trajectory: trajectory_pb2.ScalarTrajectory = dicrete_trajectory_1d(
        duration=20, dt=0.1, trajectory_function=continuous_trajectory_1d
    )
    claw_gripper_request = gripper_command_pb2.ClawGripperCommand.Request(trajectory=gripper_trajectory)
    gripper_request = gripper_command_pb2.GripperCommand.Request(claw_gripper_command=claw_gripper_request)

    # Build a Robot Command.

    synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
        mobility_command=mobility_request, arm_command=arm_request, gripper_command=gripper_request
    )
    command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
    return command


def is_batch_required(command: robot_command_pb2.RobotCommand, batch_size: int):

    command = build_robot_command()

    # Check conditions.

    batch_size = 50
    long_trajectories = []

    if command.HasField("synchronized_command"):
        if command.synchronized_command.HasField("mobility_command"):
            mobility_request = command.synchronized_command.mobility_command
            trajectory = mobility_request.se2_trajectory_request.trajectory
            if len(trajectory.points) > batch_size:
                long_trajectories.append(trajectory.points)
        if command.synchronized_command.HasField("arm_command"):
            arm_request = command.synchronized_command.arm_command
            if arm_request.HasField("arm_cartesian_command"):
                trajectory = arm_request.arm_cartesian_command.pose_trajectory_in_task
                if len(trajectory.points) > batch_size:
                    long_trajectories.append(trajectory.points)
            elif arm_request.HasField("arm_joint_move_command"):
                trajectory = arm_request.arm_joint_move_command.trajectory
                if len(trajectory.points) > batch_size:
                    long_trajectories.append(trajectory.points)
            elif arm_request.HasField("arm_impedance_command"):
                trajectory = arm_request.arm_impedance_command.task_tform_desired_tool
                if len(trajectory.points) > batch_size:
                    long_trajectories.append(trajectory.points)
        if command.synchronized_command.HasField("gripper_command"):
            gripper_request = command.synchronized_command.gripper_command
            trajectory = gripper_request.claw_gripper_command.trajectory
            if len(trajectory.points) > batch_size:
                long_trajectories.append(trajectory.points)

    if len(long_trajectories) < 1:
        return False
    elif len(long_trajectories) == 1:
        return True

    # If there are more trajectories longer than the batch size, we must
    # check if they are aligned.

    return False


def test_tmp():
    """
    Ongoing prototype.
    """
    batch_size = 50
    command = build_robot_command()
    value: bool = is_batch_required(command, batch_size)
    print(value)
