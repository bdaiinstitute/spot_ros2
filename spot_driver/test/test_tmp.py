"""
Working with trajectories.
"""

import math
import time
from typing import Callable, Tuple

from bosdyn.api import trajectory_pb2
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose, SE3Velocity
from bosdyn.util import seconds_to_duration, seconds_to_timestamp
from bosdyn.client.robot_command import RobotCommandBuilder

TrajectoryFunction3D = Callable[[float], Tuple[SE3Pose, SE3Velocity]]
TrajectoryFunction2D = Callable[[float], SE2Pose]


def gripper_trajectory(t: float) -> float:
    pass


def mobility_trajectory(t: float) -> SE2Pose:
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
    return SE2Pose(x, y, angle)


def cartesian_trajectory(t: float) -> Tuple[SE3Pose, SE3Velocity]:
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


def s2_trajectory(
    duration: float, dt: float, trajectory_function: TrajectoryFunction3D
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
        pos = trajectory_function(t)
        point = trajectory.points.add()
        point.pose.CopyFrom(pos.to_proto())
        point.time_since_reference.CopyFrom(seconds_to_duration(t - start_time))
        t = t + dt
    return trajectory


def s3_trajectory(
    duration: float, dt: float, trajectory_function: TrajectoryFunction2D
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


def test_tmp():
    trajectory_3d: trajectory_pb2.SE3Trajectory = s3_trajectory(
        duration=20, dt=0.1, trajectory_function=cartesian_trajectory
    )
    trajectory_2d: trajectory_pb2.SE2Trajectory = s2_trajectory(
        duration=20, dt=0.1, trajectory_function=mobility_trajectory
    )

    RobotCommandBuilder.body_pose

    print("OK")
