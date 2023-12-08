#!/usr/bin/python3

# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import argparse

import numpy as np

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
import bosdyn_msgs.msg
import geometry_msgs.msg
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME, ODOM_FRAME_NAME
from spot_utilities.spot_basic import SpotBasic
from utilities.tf_listener_wrapper import TFListenerWrapper

from bosdyn.client.math_helpers import Quat, SE3Pose

import spot_msgs.srv


def create_kinematic_request(
    odom_T_task: SE3Pose, wr1_T_tool: SE3Pose, task_T_desired_tool: SE3Pose
) -> spot_msgs.srv.GetInverseKinematicSolutions.Request:

    # Task frame.
    task_frame = geometry_msgs.msg.Pose()
    task_frame.position.x = 0.0
    task_frame.position.y = 0.0
    task_frame.position.z = 0.0
    task_frame.orientation.w = 1.0
    task_frame.orientation.x = 0.0
    task_frame.orientation.y = 0.0
    task_frame.orientation.z = 0.0

    # Tool frame.
    tools_specification = bosdyn_msgs.msg.InverseKinematicsRequestOneOfToolSpecification()
    tools_specification.tool_specification_choice = (
        bosdyn_msgs.msg.InverseKinematicsRequestOneOfToolSpecification.TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET
    )
    tools_specification.wrist_mounted_tool.wrist_tform_tool_is_set = True
    tools_specification.wrist_mounted_tool.wrist_tform_tool.position.x = 0.0
    tools_specification.wrist_mounted_tool.wrist_tform_tool.position.y = 0.0
    tools_specification.wrist_mounted_tool.wrist_tform_tool.position.z = 0.0
    tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.w = 1.0
    tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.x = 0.0
    tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.y = 0.0
    tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.z = 0.0

    # Task specificarion.
    task_specification = bosdyn_msgs.msg.InverseKinematicsRequestOneOfTaskSpecification()
    task_specification.task_specification_choice = (
        bosdyn_msgs.msg.InverseKinematicsRequestOneOfTaskSpecification.TASK_SPECIFICATION_TOOL_POSE_TASK_SET
    )
    task_specification.tool_pose_task.task_tform_desired_tool_is_set = True
    task_specification.tool_pose_task.task_tform_desired_tool.position.x = 0.0
    task_specification.tool_pose_task.task_tform_desired_tool.position.y = 0.0
    task_specification.tool_pose_task.task_tform_desired_tool.position.z = 0.0
    task_specification.tool_pose_task.task_tform_desired_tool.orientation.w = 1.0
    task_specification.tool_pose_task.task_tform_desired_tool.orientation.x = 0.0
    task_specification.tool_pose_task.task_tform_desired_tool.orientation.y = 0.0
    task_specification.tool_pose_task.task_tform_desired_tool.orientation.z = 0.0

    # Request.
    request = bosdyn_msgs.msg.InverseKinematicsRequest()
    request.root_frame_name = "odom"
    request.scene_tform_task_is_set = True
    request.scene_tform_task = task_frame
    request.tool_specification = tools_specification
    request.task_specification = task_specification

    result = spot_msgs.srv.GetInverseKinematicSolutions.Request()
    result.request = request

    return result


def send_requests(robot_name: str, poses: int) -> bool:

    # Set up basic ROS2 utilities for communicating with the driver.
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    logger = node.get_logger()

    # Frame names.
    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    flat_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    ground_plane_frame_name = namespace_with(robot_name, GROUND_PLANE_FRAME_NAME)

    # Wait for the robot to publish the TF state.
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, flat_body_frame_name)

    # Instantiate a facade to send commands to the robot.
    robot = SpotBasic(robot_name)

    # Claim robot.
    logger.info("Claiming robot")
    result = robot.claim()
    if not result:
        node.get_logger().error("Unable to claim robot")
        return False
    logger.info("Claimed robot")

    # Power on robot.
    logger.info("Powering robot on")
    result = robot.power_on()
    if not result:
        logger.error("Unable to power on robot")
        return False

    # Stand up robot.
    logger.info("Standing robot up")
    result = robot.stand()
    if not result:
        logger.error("Robot did not stand")
        return False
    logger.info("Successfully stood up.")

    ############################################

    odom_T_flat_body: SE3Pose = tf_listener.lookup_a_tform_b(odom_frame_name, flat_body_frame_name)
    odom_T_gpe: SE3Pose = tf_listener.lookup_a_tform_b(odom_frame_name, ground_plane_frame_name)

    # Construct the frame on the ground right underneath the center of the body.
    odom_T_ground_body: SE3Pose = odom_T_flat_body
    odom_T_ground_body.z = odom_T_gpe.z

    # Now, construct a task frame slightly above the ground, in front of the robot.
    odom_T_task: SE3Pose = odom_T_ground_body * SE3Pose(x=0.4, y=0, z=0.05, rot=Quat(w=1, x=0, y=0, z=0))

    # Now, let's set our tool frame to be the tip of the robot's bottom jaw. Flip the
    # orientation so that when the hand is pointed downwards, the tool's z-axis is
    # pointed upward.
    wr1_T_tool: SE3Pose = SE3Pose(0.23589, 0, -0.03943, Quat.from_pitch(-np.pi / 2))

    # Generate several random poses relative to the task frame.
    rng = np.random.seed(0)
    x_size = 0.7  # m
    y_size = 0.8  # m
    x_rt_task = x_size * rng.random(poses)
    y_rt_task = -y_size / 2 + y_size * rng.random(poses)
    task_T_desired_tools = [
        SE3Pose(xi_rt_task, yi_rt_task, 0.0, Quat())
        for (xi_rt_task, yi_rt_task) in zip(x_rt_task.flatten(), y_rt_task.flatten())
    ]
    task_T_desired_tools = task_T_desired_tools[0]

    # Create a target pose.
    task_T_desired_tool = SE3Pose(0, 0, 0, Quat.from_pitch(-np.pi / 2))
    request = create_kinematic_request(
        odom_T_task=odom_T_task, wr1_T_tool=wr1_T_tool, task_T_desired_tool=task_T_desired_tool
    )

    # Send inverse kinematic request.
    kinematics_client = node.create_client(
        spot_msgs.srv.GetInverseKinematicSolutions, namespace_with(robot_name, "get_inverse_kinematic_solutions")
    )
    if not kinematics_client.wait_for_service():
        logger.info("Service get_inverse_kinematics_solutions not available.")
        return False

    response = kinematics_client.call(request)

    # Power off robot.
    logger.info("Powering robot off")
    result = robot.power_off()
    if not result:
        logger.error("Unable to power off robot")
        return False

    return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="The robot name.")
    parser.add_argument("-n", "--poses", type=int, default=50, help="Number of desired tool poses to query.")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    send_requests(args.robot, args.poses)


if __name__ == "__main__":
    main()
