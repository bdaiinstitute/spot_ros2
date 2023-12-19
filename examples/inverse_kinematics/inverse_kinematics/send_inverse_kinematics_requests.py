#!/usr/bin/python3

# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import argparse
import time
from typing import List

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
import bosdyn_msgs.msg
import geometry_msgs.msg
import numpy as np
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import geometry_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    ODOM_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from spatialmath import SE2
from spot_utilities.spot_basic import SpotBasic
from tf2_ros import TransformBroadcaster
from utilities.tf_listener_wrapper import TFListenerWrapper

import spot_driver.conversions as conv
import spot_msgs.srv
from spot_msgs.action import RobotCommand  # type: ignore


class IKTest:
    def __init__(self, node: Node, args: argparse.Namespace):
        self.node = node
        self.robot_name: str = args.robot
        self.docking_station: int = args.dock
        self.poses: int = args.poses
        self.logger = node.get_logger()
        self.tf_broadcaster = TransformBroadcaster(node)
        self.tf_listener = TFListenerWrapper(node)
        self.robot = SpotBasic(self.robot_name)
        self.ik_client = node.create_client(
            spot_msgs.srv.GetInverseKinematicSolutions,
            namespace_with(self.robot_name, "get_inverse_kinematic_solutions"),
        )
        self.robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self.robot_name, "robot_command"), node
        )

        self.timer = node.create_timer(0.1, self.timer_callback)
        self.transforms: List[TransformStamped] = []

    def publish_transform(self, parent_frame_name: str, child_frame_name: str, pose: SE3Pose) -> None:
        """
        Create a transformation and add it to a list so we can display it later in RViz.
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

    def timer_callback(self) -> None:
        """
        Publish all cached tranformations at 10Hz so we can view
        """
        for tf in self.transforms:
            tf.header.stamp = self.node.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(tf)

    def create_ik_request(
        self, odom_T_task: SE3Pose, wr1_T_tool: SE3Pose, task_T_desired_tool: SE3Pose
    ) -> spot_msgs.srv.GetInverseKinematicSolutions.Request:
        """
        Create an inverse kinematic request.

        Args:
            odom_T_task The task frame relative to the odom frame.
            wr1_T_tool The tool frame relative to the wrist frame.
            task_T_desired_tool The desired position of the tool frame relative to the task frame.

        Returns:
            A ROS2 IK request.
        """

        # Task frame.
        task_frame = geometry_msgs.msg.Pose()
        task_frame.position.x = float(odom_T_task.x)
        task_frame.position.y = float(odom_T_task.y)
        task_frame.position.z = float(odom_T_task.z)
        task_frame.orientation.w = float(odom_T_task.rotation.w)
        task_frame.orientation.x = float(odom_T_task.rotation.x)
        task_frame.orientation.y = float(odom_T_task.rotation.y)
        task_frame.orientation.z = float(odom_T_task.rotation.z)

        # Tool frame.
        tools_specification = bosdyn_msgs.msg.InverseKinematicsRequestOneOfToolSpecification()
        tools_specification.tool_specification_choice = (
            bosdyn_msgs.msg.InverseKinematicsRequestOneOfToolSpecification.TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET
        )
        tools_specification.wrist_mounted_tool.wrist_tform_tool_is_set = True
        tools_specification.wrist_mounted_tool.wrist_tform_tool.position.x = float(wr1_T_tool.x)
        tools_specification.wrist_mounted_tool.wrist_tform_tool.position.y = float(wr1_T_tool.y)
        tools_specification.wrist_mounted_tool.wrist_tform_tool.position.z = float(wr1_T_tool.z)
        tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.w = float(wr1_T_tool.rotation.w)
        tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.x = float(wr1_T_tool.rotation.x)
        tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.y = float(wr1_T_tool.rotation.y)
        tools_specification.wrist_mounted_tool.wrist_tform_tool.orientation.z = float(wr1_T_tool.rotation.z)

        # Task specificarion.
        task_specification = bosdyn_msgs.msg.InverseKinematicsRequestOneOfTaskSpecification()
        task_specification.task_specification_choice = (
            bosdyn_msgs.msg.InverseKinematicsRequestOneOfTaskSpecification.TASK_SPECIFICATION_TOOL_POSE_TASK_SET
        )
        task_specification.tool_pose_task.task_tform_desired_tool_is_set = True
        task_specification.tool_pose_task.task_tform_desired_tool.position.x = float(task_T_desired_tool.x)
        task_specification.tool_pose_task.task_tform_desired_tool.position.y = float(task_T_desired_tool.y)
        task_specification.tool_pose_task.task_tform_desired_tool.position.z = float(task_T_desired_tool.z)
        task_specification.tool_pose_task.task_tform_desired_tool.orientation.w = float(task_T_desired_tool.rotation.w)
        task_specification.tool_pose_task.task_tform_desired_tool.orientation.x = float(task_T_desired_tool.rotation.x)
        task_specification.tool_pose_task.task_tform_desired_tool.orientation.y = float(task_T_desired_tool.rotation.y)
        task_specification.tool_pose_task.task_tform_desired_tool.orientation.z = float(task_T_desired_tool.rotation.z)

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
        namespace_with(self.robot_name, "desired_pose")

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

        # Walk forward 1.2m.
        self.logger.info("Walking forward")
        result = self.robot.walk_to(SE2(1.2, 0, 0))
        if not result:
            self.logger.error("Cannot walk forward")
            return False
        self.logger.info("Successfully walked forward.")

        # Rotate 90 degrees left.
        self.logger.info("Rotate 90 degrees")
        result = self.robot.walk_to(SE2(1.2, 0, 1.5708))
        if not result:
            self.logger.error("Cannot rotate")
            return False
        self.logger.info("Successfully rotated 90 degrees.")

        # Look for known transforms published by the robot.
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

        # Generate several random poses in front of the task frame where we want the tool to move to.
        # The desired tool poses are defined relative to thr task frame in front of the robot and slightly
        # above the ground. The task frame is aligned with the "gravity aligned body frame", such that
        # the positive-x direction is to the front of the robot, the positive-y direction is to the left
        # of the robot, and the positive-z direction is opposite to gravity.
        x_size = 0.8  # m
        y_size = 0.8  # m
        x_rt_task = x_size * np.random.random(self.poses)
        y_rt_task = -y_size / 2 + y_size * np.random.random(self.poses)
        task_T_desired_tools = [
            SE3Pose(xi_rt_task, yi_rt_task, 0.0, Quat())
            for (xi_rt_task, yi_rt_task) in zip(x_rt_task.flatten(), y_rt_task.flatten())
        ]

        # Define a stand command that we'll send if the IK service does not find a solution.
        body_control = spot_command_pb2.BodyControlParams(
            body_assist_for_manipulation=spot_command_pb2.BodyControlParams.BodyAssistForManipulation(
                enable_hip_height_assist=True, enable_body_yaw_assist=True
            )
        )
        body_assist_enabled_stand_command = RobotCommandBuilder.synchro_stand_command(
            params=spot_command_pb2.MobilityParams(body_control=body_control)
        )

        # Unstow the arm.
        self.logger.info("Unstow the arm")
        result = self.robot.ready_arm()
        if not result:
            self.logger.error("Failed to unstow the arm.")
            return False
        self.logger.info("Arm ready.")

        # Check if the IK service is available.
        if not self.ik_client.wait_for_service():
            self.logger.info("Service get_inverse_kinematics_solutions not available.")
            return False

        # Send IK requests.
        for i, task_T_desired_tool in enumerate(task_T_desired_tools):
            ik_request = self.create_ik_request(
                odom_T_task=odom_T_task, wr1_T_tool=wr1_T_tool, task_T_desired_tool=task_T_desired_tool
            )
            ik_response: spot_msgs.srv.GetInverseKinematicSolutions.Response = self.ik_client.call(ik_request)

            # Attempt to move to each of the desired tool pose to check the IK results.
            stand_command = None
            if ik_response.response.status.value == bosdyn_msgs.msg.InverseKinematicsResponseStatus.STATUS_OK:
                self.logger.info("Solution found")
                self.publish_transform(task_frame_name, "T" + str(i) + "-YES", task_T_desired_tool)

                # We don't have yet a ROS2 method to get a transform from a snapshot:
                # we must convert the ROS2 message into Protobuf first.
                frame_tree_snapshot = geometry_pb2.FrameTreeSnapshot()
                conv.convert_bosdyn_msgs_frame_tree_snapshot_to_proto(
                    ik_response.response.robot_configuration.transforms_snapshot, frame_tree_snapshot
                )
                odom_T_desired_body = get_a_tform_b(
                    frame_tree_snapshot,
                    ODOM_FRAME_NAME,
                    BODY_FRAME_NAME,
                )
                mobility_params = spot_command_pb2.MobilityParams(
                    body_control=spot_command_pb2.BodyControlParams(
                        body_pose=RobotCommandBuilder.body_pose(ODOM_FRAME_NAME, odom_T_desired_body.to_proto())
                    )
                )
                stand_command = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
            elif (
                ik_response.response.status.value
                == bosdyn_msgs.msg.InverseKinematicsResponseStatus.STATUS_NO_SOLUTION_FOUND
            ):
                self.logger.info("No solution found")
                self.publish_transform(task_frame_name, "T" + str(i) + "-NO", task_T_desired_tool)
                stand_command = body_assist_enabled_stand_command
            else:
                self.logger.info("Status unknown")
                self.publish_transform(task_frame_name, "T" + str(i) + "-NO", task_T_desired_tool)
                stand_command = body_assist_enabled_stand_command

            # Move the arm tool to the requested position.
            arm_command = RobotCommandBuilder.arm_pose_command_from_pose(
                hand_pose=(odom_T_task * task_T_desired_tool).to_proto(),
                frame_name=ODOM_FRAME_NAME,
                seconds=1,
                build_on_command=stand_command,
            )
            arm_command.synchronized_command.arm_command.arm_cartesian_command.wrist_tform_tool.CopyFrom(
                wr1_T_tool.to_proto()
            )
            arm_command_goal = RobotCommand.Goal()
            conv.convert_proto_to_bosdyn_msgs_robot_command(arm_command, arm_command_goal.command)
            result = self.robot_command_client.send_goal_and_wait(
                action_name="arm_move_one", goal=arm_command_goal, timeout_sec=5
            )

        # Dock robot.
        self.logger.info("Docking the robot")
        result = self.robot.dock(self.docking_station)
        if not result:
            self.logger.error("Unable to dock the robot")
            return False

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
    parser.add_argument("--dock", type=int, required=True, help="The docking station number (527 for Spot Opal).")
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
