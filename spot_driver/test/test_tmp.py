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
from bosdyn.util import duration_to_seconds, seconds_to_duration, seconds_to_timestamp

ContinuousTrajectory1D = Callable[[float], float]
ContinuousTrajectory2D = Callable[[float], Tuple[SE2Pose, SE2Velocity]]
ContinuousTrajectory3D = Callable[[float], Tuple[SE3Pose, SE3Velocity]]

logging.basicConfig(level=logging.INFO)

###############################################################################
# CONTINOUS TRAJECTORIES
# Here we define some continuous functions to be used to generate
# sample trajectories.


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


###############################################################################
# DISCRETE TRAJECTORIES
# Here we define some trajectories by sampling continuous functions.


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


###############################################################################
# ROBOT COMMAND
# Here we build a robot command containing different trajectories generated
# from the previously defined samples.


def build_sample_command() -> robot_command_pb2.RobotCommand:
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


###############################################################################
# LOGIC
# Here is the actual logic that we want to implement to:
# 1 - Check if a robot command contains trajectories to be batched;
# 2 - Create a sequence of robot commands.


def is_batch_required(command: robot_command_pb2.RobotCommand, batch_size: int):
    """
    This method returns true if the given command contains trajectories that
    can be batched, false otherwise. To be batched, the command must contain
    only one trajectory longer than the batch size or multiple trajectories
    that are also time aligned.
    """

    # Find all trajectories that require batching.

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

    long_trajectories_count = len(long_trajectories)
    if long_trajectories_count < 1:
        return False
    if long_trajectories_count == 1:
        return True

    # Check that all long trajectories have the same size.
    long_trajectory_size = len(long_trajectories[0])
    same_size = all(len(trajectory) == long_trajectory_size for trajectory in long_trajectories)
    if not same_size:
        return False

    # Check that all long trajectories to are time aligned.
    for i in range(long_trajectory_size):
        time_since_reference = duration_to_seconds(long_trajectories[0][i].time_since_reference)
        for j in range(1, len(long_trajectories)):
            if not duration_to_seconds(long_trajectories[j][i].time_since_reference) == time_since_reference:
                return False

    return True


def slice_trajectory(trajectory, index: int, batch_size: int) -> bool:
    """
    This command slices the trajectory protobuf message.
    """
    completed = True
    batch = trajectory.points[index : index + batch_size]
    if len(trajectory.points) - len(batch) - index > 0:
        completed = False
    trajectory.ClearField("points")
    trajectory.points.extend(batch)
    return completed


def batch_command(command: robot_command_pb2.RobotCommand, batch_size: int) -> list[robot_command_pb2.RobotCommand]:
    """
    Analyze the trajectories inside the given command and if they require
    batching, then return an equivalend sequence of commands.
    """

    if not is_batch_required(command, batch_size):
        return [command]

    index = 0
    commands: list[robot_command_pb2.RobotCommand] = []

    completed = False
    while not completed:
        completed = True

        new_command = robot_command_pb2.RobotCommand()
        new_command.CopyFrom(command)

        if new_command.HasField("synchronized_command"):
            if new_command.synchronized_command.HasField("mobility_command"):
                mobility_request = new_command.synchronized_command.mobility_command
                trajectory = mobility_request.se2_trajectory_request.trajectory
                completed = completed and slice_trajectory(trajectory, index, batch_size)
            if new_command.synchronized_command.HasField("arm_command"):
                arm_request = new_command.synchronized_command.arm_command
                if arm_request.HasField("arm_cartesian_command"):
                    trajectory = arm_request.arm_cartesian_command.pose_trajectory_in_task
                    completed = completed and slice_trajectory(trajectory, index, batch_size)
                elif arm_request.HasField("arm_joint_move_command"):
                    trajectory = arm_request.arm_joint_move_command.trajectory
                    completed = completed and slice_trajectory(trajectory, index, batch_size)
                elif arm_request.HasField("arm_impedance_command"):
                    trajectory = arm_request.arm_impedance_command.task_tform_desired_tool
                    completed = completed and slice_trajectory(trajectory, index, batch_size)
            if new_command.synchronized_command.HasField("gripper_command"):
                gripper_request = new_command.synchronized_command.gripper_command
                trajectory = gripper_request.claw_gripper_command.trajectory
                completed = completed and slice_trajectory(trajectory, index, batch_size)

        commands.append(new_command)
        index += batch_size

    return commands


def test_tmp():
    """
    Ongoing prototype.
    """
    batch_size = 50
    command = build_sample_command()
    commands = batch_command(command, batch_size)
    print(len(commands))
