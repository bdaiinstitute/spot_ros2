# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Utility class with methods to manipulate robot commands.
"""
from typing import Any, List

from bosdyn.api import robot_command_pb2
from bosdyn.util import duration_to_seconds


def should_batch(command: robot_command_pb2.RobotCommand, batch_size: int) -> bool:
    """
    This method returns true if the given command contains trajectories that
    can be batched, false otherwise. To be batched, the command must contain
    only one trajectory longer than the batch size or multiple trajectories
    that are also time aligned.

    Args:
    command: A robot command with some trajectories.
    batch_size: A batch size

    Returns:
        If the command has no trajectory is longer than the given batch size,
        then returns false.
        If the command has one trajectory longer than the given batch size, or
        multiple trajectories which are also time aligned, then return true.
    """

    # Find all trajectories that require batching.

    long_trajectories = []

    if command.HasField("synchronized_command"):
        if command.synchronized_command.HasField("mobility_command"):
            mobility_request = command.synchronized_command.mobility_command
            points = mobility_request.se2_trajectory_request.trajectory.points
            if len(points) > batch_size:
                long_trajectories.append(points)
        if command.synchronized_command.HasField("arm_command"):
            arm_request = command.synchronized_command.arm_command
            if arm_request.HasField("arm_cartesian_command"):
                points = arm_request.arm_cartesian_command.pose_trajectory_in_task.points
                if len(points) > batch_size:
                    long_trajectories.append(points)
            elif arm_request.HasField("arm_joint_move_command"):
                points = arm_request.arm_joint_move_command.trajectory.points
                if len(points) > batch_size:
                    long_trajectories.append(points)
            elif arm_request.HasField("arm_impedance_command"):
                points = arm_request.arm_impedance_command.task_tform_desired_tool.points
                if len(points) > batch_size:
                    long_trajectories.append(points)
        if command.synchronized_command.HasField("gripper_command"):
            gripper_request = command.synchronized_command.gripper_command
            points = gripper_request.claw_gripper_command.trajectory.points
            if len(points) > batch_size:
                long_trajectories.append(points)

    long_trajectories_count = len(long_trajectories)

    # No trajectory to batch.
    if long_trajectories_count < 1:
        return False

    # One trajectory to batch.
    if long_trajectories_count == 1:
        return True

    # There is more than one trajectory longer than the batch size.

    # Check that all long trajectories have the same size.
    long_trajectory_size = len(long_trajectories[0])
    same_size = all(len(trajectory) == long_trajectory_size for trajectory in long_trajectories)
    if not same_size:
        return False

    # Check that all long trajectories to are all time aligned.
    for i in range(long_trajectory_size):
        time_since_reference = duration_to_seconds(long_trajectories[0][i].time_since_reference)
        for j in range(1, len(long_trajectories)):
            if not duration_to_seconds(long_trajectories[j][i].time_since_reference) == time_since_reference:
                return False

    return True


def slice_trajectory(trajectory: Any, index: int, batch_size: int) -> bool:
    """
    This command expects a trajectory protobuf message containing a repeated
    field "points" that we want to slice.

    Args:
        trajectory: A protobuf message containing a repeated field "points" that we want to slice.
        index: The position to slice the field from.
        batch_size: The length of the slice.

    Returns:
        True if the last batch has been extracted.

    """
    is_last_batch = True
    batch = trajectory.points[index : index + batch_size]
    if len(trajectory.points) - len(batch) - index > 0:
        is_last_batch = False
    trajectory.ClearField("points")
    trajectory.points.extend(batch)
    return is_last_batch


def batch_command(
    command: robot_command_pb2.RobotCommand, batch_size: int, overlapping: int = 0
) -> List[robot_command_pb2.RobotCommand]:
    """
    Analyze the trajectories inside the given command and if they require
    batching, then return an equivalent sequence of commands.

    Args:
        command: A robot command with some trajectories.
        batch_size: A batch size
        overlapping: Number of points that must overlap between batched
            trajectories.

    Returns:
        If no trajectory is longer than the given batch size, then returns
        an array containing the same robot command.
        If it contains one trajectory longer than the given batch size, or
        multiple trajectories which are also time aligned, then return an
        array of robot commands, each of them representing a batch.
    """

    if not should_batch(command, batch_size):
        return [command]

    # This is the increment to find the position of the next batch.
    stride = batch_size - overlapping
    if stride < 1:
        return [command]

    index = 0
    commands: List[robot_command_pb2.RobotCommand] = []

    is_last_batch = False
    while not is_last_batch:
        is_last_batch = True

        new_command = robot_command_pb2.RobotCommand()
        new_command.CopyFrom(command)

        if new_command.HasField("synchronized_command"):
            if new_command.synchronized_command.HasField("mobility_command"):
                mobility_request = new_command.synchronized_command.mobility_command
                trajectory = mobility_request.se2_trajectory_request.trajectory
                is_last_batch = slice_trajectory(trajectory, index, batch_size) and is_last_batch
            if new_command.synchronized_command.HasField("arm_command"):
                arm_request = new_command.synchronized_command.arm_command
                if arm_request.HasField("arm_cartesian_command"):
                    trajectory = arm_request.arm_cartesian_command.pose_trajectory_in_task
                    is_last_batch = slice_trajectory(trajectory, index, batch_size) and is_last_batch
                elif arm_request.HasField("arm_joint_move_command"):
                    trajectory = arm_request.arm_joint_move_command.trajectory
                    is_last_batch = slice_trajectory(trajectory, index, batch_size) and is_last_batch
                elif arm_request.HasField("arm_impedance_command"):
                    trajectory = arm_request.arm_impedance_command.task_tform_desired_tool
                    is_last_batch = slice_trajectory(trajectory, index, batch_size) and is_last_batch
            if new_command.synchronized_command.HasField("gripper_command"):
                gripper_request = new_command.synchronized_command.gripper_command
                trajectory = gripper_request.claw_gripper_command.trajectory
                is_last_batch = slice_trajectory(trajectory, index, batch_size) and is_last_batch

        commands.append(new_command)
        index += stride

    return commands


def min_time_since_reference(command: robot_command_pb2.RobotCommand) -> float:
    """
    This method inspects all trajectories in the command and returns the minimum
    time_since_reference.
    It returns float("inf") if no trajectory is available.

    Args:
        command: The command to inspect.

    Returns:
        The minimum time_since_reference of all trajectories in the command.
    """
    min_time_since_reference = float("inf")
    if command.HasField("synchronized_command"):
        if command.synchronized_command.HasField("mobility_command"):
            mobility_request = command.synchronized_command.mobility_command
            if trajectory := mobility_request.se2_trajectory_request.trajectory:
                time = duration_to_seconds(trajectory.points[0].time_since_reference)
                if time < min_time_since_reference:
                    min_time_since_reference = time
        if command.synchronized_command.HasField("arm_command"):
            arm_request = command.synchronized_command.arm_command
            if arm_request.HasField("arm_cartesian_command"):
                if trajectory := arm_request.arm_cartesian_command.pose_trajectory_in_task:
                    time = duration_to_seconds(trajectory.points[0].time_since_reference)
                    if time < min_time_since_reference:
                        min_time_since_reference = time
            elif arm_request.HasField("arm_joint_move_command"):
                if trajectory := arm_request.arm_joint_move_command.trajectory:
                    time = duration_to_seconds(trajectory.points[0].time_since_reference)
                    if time < min_time_since_reference:
                        min_time_since_reference = time
            elif arm_request.HasField("arm_impedance_command"):
                if trajectory := arm_request.arm_impedance_command.task_tform_desired_tool:
                    time = duration_to_seconds(trajectory.points[0].time_since_reference)
                    if time < min_time_since_reference:
                        min_time_since_reference = time
        if command.synchronized_command.HasField("gripper_command"):
            gripper_request = command.synchronized_command.gripper_command
            if trajectory := gripper_request.claw_gripper_command.trajectory:
                time = duration_to_seconds(trajectory.points[0].time_since_reference)
                if time < min_time_since_reference:
                    min_time_since_reference = time
    return min_time_since_reference


def get_batch_size(sequence_length: int, batch_size: int, overlapping: int, batch_number: int) -> int:
    """
    Return the size of a batch considering a vector of given length, the batch size
    and the overlapping points.

    Args:
        sequence_length: The sequence length.
        batch_size: The batch size.
        overlapping: The number of overlapping element between batches.
        batch_number: The batch number.

    Returns:
        The size of the requested batch.

    Examples:

        sequence: 0 1 2 3 4 5 6 7 8 9
        sequence_length = 10
        batch_size = 5
        overlapping = 0
        batch1:   0 1 2 3 4
        batch2:             5 6 7 8 9
        batch sizes = [5, 5]

        sequence: 0 1 2 3 4 5 6 7 8 9 A B C
        sequence_length = 13
        batch_size = 5
        overlapping = 1
        batch1:   0 1 2 3 4
        batch2:           4 5 6 7 8
        batch3:                   8 9 A B C
        batch4:                           C
        batch sizes = [5, 5, 5, 1]
    """
    # Calculate the stride.
    stride = batch_size - overlapping

    # Calculate the total number of full batches.
    num_full_batches = max(0, 1 + (sequence_length - batch_size) // stride)

    if batch_number >= num_full_batches:
        # Calculate the remaining elements.
        return max(0, sequence_length - stride * batch_number)
    else:
        return batch_size
