#!/usr/bin/python3

# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import argparse
import time

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
import bosdyn_msgs.msg
import geometry_msgs.msg
import numpy as np
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from spot_utilities.spot_basic import SpotBasic
from tf2_ros import TransformBroadcaster
from utilities.tf_listener_wrapper import TFListenerWrapper

import spot_msgs.srv


class IKTest:
    def __init__(self, node: Node, args: argparse.Namespace):
        self.node = node
        self.robot_name: str = args.robot
        self.poses: int = args.poses
        self.logger = node.get_logger()
        self.tf_broadcaster = TransformBroadcaster(node)
        self.tf_listener = TFListenerWrapper(node)
        self.robot = SpotBasic(self.robot_name)
        self.kinematics_client = node.create_client(
            spot_msgs.srv.GetInverseKinematicSolutions,
            namespace_with(self.robot_name, "get_inverse_kinematic_solutions"),
        )
        self.timer = node.create_timer(0.1, self.timer_callback)
        self.transforms = []

    def publish_transform(self, parent_frame_name: str, child_frame_name: str, pose: SE3Pose) -> None:
        """
        Cache a transform.
        """
        tf = TransformStamped()
        tf.header.frame_id = parent_frame_name
        tf.child_frame_id = child_frame_name
        tf.transform.translation.x = float(pose.x)
        tf.transform.translation.y = float(pose.y)
        tf.transform.translation.z = float(pose.z)
        tf.transform.rotation.x = float(pose.rot.x)
        tf.transform.rotation.y = float(pose.rot.y)
        tf.transform.rotation.z = float(pose.rot.z)
        tf.transform.rotation.w = float(pose.rot.w)
        self.transforms.append(tf)

    def timer_callback(self):
        """
        Publish all cached tranformations at 10Hz.
        """
        for tf in self.transforms:
            tf.header.stamp = self.node.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(tf)

    def create_kinematic_request(
        self, odom_T_task: SE3Pose, wr1_T_tool: SE3Pose, task_T_desired_tool: SE3Pose
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

    def send_requests(self) -> bool:

        # Frame names.
        odom_frame_name = namespace_with(self.robot_name, ODOM_FRAME_NAME)
        flat_body_frame_name = namespace_with(self.robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        ground_plane_frame_name = namespace_with(self.robot_name, GROUND_PLANE_FRAME_NAME)

        task_frame_name = namespace_with(self.robot_name, "task_frame")
        link_wr1_frame_name = namespace_with(self.robot_name, "link_wr1")
        jaw_frame_name = namespace_with(self.robot_name, "jaw_frame")
        desired_pose_name = namespace_with(self.robot_name, "desired_pose")

        # Wait for the robot to publish the TF state.
        self.tf_listener.wait_for_a_tform_b(odom_frame_name, flat_body_frame_name)

        # Claim robot.
        self.logger.info("Claiming robot")
        result = self.robot.claim()
        if not result:
            self.logger.error("Unable to claim robot")
            return False
        self.logger.info("Claimed robot")

        # Power on robot.
        self.logger.info("Powering robot on")
        result = self.robot.power_on()
        if not result:
            self.logger.error("Unable to power on robot")
            return False

        # Stand up robot.
        self.logger.info("Standing robot up")
        result = self.robot.stand()
        if not result:
            self.logger.error("Robot did not stand")
            return False
        self.logger.info("Successfully stood up.")

        # Look for known transforms.

        odom_T_flat_body: SE3Pose = self.tf_listener.lookup_a_tform_b(odom_frame_name, flat_body_frame_name)
        odom_T_gpe: SE3Pose = self.tf_listener.lookup_a_tform_b(odom_frame_name, ground_plane_frame_name)

        # Construct the frame on the ground right underneath the center of the body.
        odom_T_ground_body: SE3Pose = odom_T_flat_body
        odom_T_ground_body.z = odom_T_gpe.z

        # Now, construct a task frame slightly above the ground, in front of the robot.
        odom_T_task: SE3Pose = odom_T_ground_body * SE3Pose(x=0.4, y=0.0, z=0.05, rot=Quat(w=1.0, x=0.0, y=0.0, z=0.0))
        self.publish_transform(odom_frame_name, task_frame_name, odom_T_task)

        # Now, let's set our tool frame to be the tip of the robot's bottom jaw. Flip the
        # orientation so that when the hand is pointed downwards, the tool's z-axis is
        # pointed upward.
        wr1_T_tool: SE3Pose = SE3Pose(0.23589, 0.0, -0.03943, Quat.from_pitch(-np.pi / 2))
        self.publish_transform(link_wr1_frame_name, jaw_frame_name, wr1_T_tool)

        # Generate several random poses relative to the task frame.
        np.random.seed(0)
        x_size = 0.7  # m
        y_size = 0.8  # m
        x_rt_task = x_size * np.random.random(self.poses)
        y_rt_task = -y_size / 2 + y_size * np.random.random(self.poses)
        task_T_desired_tools = [
            SE3Pose(xi_rt_task, yi_rt_task, 0.0, Quat())
            for (xi_rt_task, yi_rt_task) in zip(x_rt_task.flatten(), y_rt_task.flatten())
        ]

        task_T_desired_tool = task_T_desired_tools[0]
        self.publish_transform(task_frame_name, desired_pose_name, task_T_desired_tool)

        request = self.create_kinematic_request(
            odom_T_task=odom_T_task, wr1_T_tool=wr1_T_tool, task_T_desired_tool=task_T_desired_tool
        )

        # Send inverse kinematic request.
        if not self.kinematics_client.wait_for_service():
            self.logger.info("Service get_inverse_kinematics_solutions not available.")
            return False

        response = self.kinematics_client.call(request)

        # Power off robot.
        self.logger.info("Powering robot off")
        result = self.robot.power_off()
        if not result:
            self.logger.error("Unable to power off robot")
            return False

        return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="The robot name.")
    parser.add_argument("-n", "--poses", type=int, default=50, help="Number of desired tool poses to query.")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:

    # Set up basic ROS2 utilities for communicating with the driver.
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")

    test = IKTest(node, args)
    test.send_requests()

    time.sleep(5)


if __name__ == "__main__":
    main()
