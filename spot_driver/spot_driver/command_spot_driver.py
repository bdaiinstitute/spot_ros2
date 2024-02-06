#!/usr/bin/env python3
import math
import time
from typing import Any, Optional

import bdai_ros2_wrappers.process as ros_process
import builtin_interfaces.msg
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Twist
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_srvs.srv import SetBool, Trigger
from tf_transformations import quaternion_from_euler

# from spot_msgs.srv import GripperAngleMove
# from spot_msgs.srv import ArmJointMovement
# from spot_msgs.srv import ArmForceTrajectory
# from spot_msgs.srv import HandPose
from spot_msgs.action import NavigateTo, Trajectory  # type: ignore
from spot_msgs.srv import ListGraph, SetLocomotion, SetVelocity  # type: ignore


class CommandSpotDriver(Node):
    def __init__(self, node_name: str, **kwargs: Any) -> None:
        try:
            super().__init__(node_name, **kwargs)
            self.declare_parameter("command", "")

            self.command = self.get_parameter("command").value

            self.claim_client = self.create_client(Trigger, "claim")
            self.release_client = self.create_client(Trigger, "release")
            self.power_on_client = self.create_client(Trigger, "power_on")
            self.power_off_client = self.create_client(Trigger, "power_off")
            self.sit_client = self.create_client(Trigger, "sit")
            self.stand_client = self.create_client(Trigger, "stand")

            self.stair_mode_client = self.create_client(SetBool, "stair_mode")
            self.set_locomotion_client = self.create_client(SetLocomotion, "locomotion_mode")
            self.set_velocity_client = self.create_client(SetVelocity, "max_velocity")
            self.list_graph_client = self.create_client(ListGraph, "list_graph")

            self.trajectory_action_client = ActionClient(self, Trajectory, "trajectory")
            self.navigate_to_action_client = ActionClient(self, NavigateTo, "navigate_to")

            self.stow_client = self.create_client(Trigger, "arm_stow")
            self.unstow_client = self.create_client(Trigger, "arm_unstow")
            self.open_client = self.create_client(Trigger, "gripper_open")
            self.close_client = self.create_client(Trigger, "gripper_close")
            self.carry_client = self.create_client(Trigger, "arm_carry")

            # self.angle_open_client = self.create_client(
            #   GripperAngleMove,
            #   "gripper_angle_open",
            #   callback_group=self.group
            # )
            # self.joint_move_client = self.create_client(
            #   ArmJointMovement,
            #   "arm_joint_move",
            #   callback_group=self.group
            # )
            # self.force_trajectory_client = self.create_client(
            #   ArmForceTrajectory,
            #   "force_trajectory",
            #   callback_group=self.group
            # )
            # self.hand_pose_client = self.create_client(
            #   HandPose,
            #   "gripper_pose",
            #   callback_group=self.group
            # )

            self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 1)
            self.body_pose_publisher = self.create_publisher(Pose, "body_pose", 1)

            self.timer = self.create_timer(0.1, self.do_command)

            self.navigate_to_get_result_future: Optional[Any] = None
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")

    def send_trajectory_goal(
        self, target_pose: PoseStamped, duration: builtin_interfaces.msg.Duration, precise_positioning: bool = False
    ) -> None:
        # PoseStamped, Duration, bool
        goal_msg = Trajectory.Goal()
        goal_msg.target_pose = target_pose
        goal_msg.duration = duration
        goal_msg.precise_positioning = precise_positioning

        self.trajectory_action_client.wait_for_server()

        self.trajectory_future = self.trajectory_action_client.send_goal_async(
            goal_msg, feedback_callback=self.trajectory_feedback_callback
        )

        self.trajectory_future.add_done_callback(self.trajectory_goal_response_callback)

    def trajectory_goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self.trajectory_get_result_future = goal_handle.get_result_async()
        self.trajectory_get_result_future.add_done_callback(self.trajectory_get_result_callback)

    def trajectory_get_result_callback(self, future: Future) -> None:
        result = future.result().result
        self.get_logger().info(f"Trajectory Result: {result.success} - {result.message}")

    def trajectory_feedback_callback(self, feedback_msg: Trajectory.Feedback) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.feedback}")

    def send_navigate_goal(self, upload_path: str, navigate_to: str) -> None:
        # PoseStamped, Duration, bool
        goal_msg = NavigateTo.Goal()
        goal_msg.upload_path = upload_path
        goal_msg.navigate_to = navigate_to
        goal_msg.initial_localization_fiducial = True
        goal_msg.initial_localization_waypoint = ""

        self.navigate_to_action_client.wait_for_server()

        self.navigate_to_future = self.navigate_to_action_client.send_goal_async(
            goal_msg, feedback_callback=self.navigate_to_feedback_callback
        )

        self.navigate_to_future.add_done_callback(self.trajectory_goal_response_callback)

    def navigate_to_goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self.navigate_to_get_result_future = goal_handle.get_result_async()
        self.navigate_to_get_result_future.add_done_callback(self.navigate_to_get_result_callback)

    def navigate_to_get_result_callback(self, future: Future) -> None:
        result = future.result().result
        self.get_logger().info(f"NavigateTo Result: {result.success} - {result.message}")

    def navigate_to_feedback_callback(self, feedback_msg: NavigateTo.Feedback) -> None:
        # feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback navigate_to: {feedback_msg}")

    def claim(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.claim_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"CLAIM: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def release(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.release_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"RELEASE: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def power_on(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.power_on_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"POWER ON: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def power_off(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.power_off_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"POWER OFF: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def sit(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.sit_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"SIT: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def stand(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.stand_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"STAND: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def list_graph(self, upload_dir: str) -> ListGraph.Response:
        try:
            self.get_logger().error(f"list_graph: {upload_dir}")
            req = ListGraph.Request()
            req.upload_filepath = upload_dir
            future = self.list_graph_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def set_locomotion(self, mode: int = 1) -> Optional[SetLocomotion.Response]:
        try:
            req = SetLocomotion.Request()
            req.locomotion_mode = mode
            future = self.set_locomotion_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"SET LOCOMOTION: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def set_velocity(self, x: float, y: float, angular: float) -> Optional[SetVelocity.Response]:
        try:
            req = SetVelocity.Request()
            req.velocity_limit.linear.x = x
            req.velocity_limit.linear.y = y
            req.velocity_limit.angular.z = angular
            future = self.set_velocity_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"SET MAX VELOCITY: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception set_velocity: {type(exc)} - {exc}")
        return None

    def stow(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.stow_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"STOW: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def unstow(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.unstow_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"UNSTOW: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def open(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.open_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"OPEN: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def close(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.close_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"CLOSE: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    def carry(self) -> Optional[Trigger.Response]:
        try:
            req = Trigger.Request()
            future = self.carry_client.call_async(req)
            if not wait_for_future(future, context=self.context):
                return None
            resp = future.result()
            self.get_logger().info(f"CARRY: {resp}")
            return resp
        except Exception as exc:
            self.get_logger().error(f"Exception: {type(exc)} - {exc}")
        return None

    # def angle_open(self):
    #     try:
    #         req = GripperAngleMove.Request()
    #         req.gripper_angle = math.radians(10.0)
    #         future = self.angle_open_client.call_async(req)
    #         resp = self.wait_future(future, print_waiting=False)
    #         self.get_logger().info(f"ANGLE OPEN: {resp}")
    #         return resp
    #     except Exception as exc:
    #         self.get_logger().error(f"Exception: {type(exc)} - {exc}")
    #     return None

    # def joint_move(self):
    #     try:
    #         req = ArmJointMove.Request()
    #         req.joint_target[0] = 0.0
    #         req.joint_target[1] = 0.0
    #         req.joint_target[2] = 0.0
    #         req.joint_target[3] = 0.0
    #         req.joint_target[4] = 0.0
    #         req.joint_target[5] = 0.0
    #         future = self.joint_move_client.call_async(req)
    #         resp = self.wait_future(future, print_waiting=False)
    #         self.get_logger().info(f"JOINT MOVE: {resp}")
    #         return resp
    #     except Exception as exc:
    #         self.get_logger().error(f"Exception: {type(exc)} - {exc}")
    #     return None

    # def force_trajectory(self):
    #     try:
    #         req = ArmForceTrajectory.Request()
    #
    #         req.forces_pt0[0] = 0.0
    #         req.forces_pt0[1] = 0.0
    #         req.forces_pt0[2] = 0.0
    #
    #         req.torques_pt0[0] = 0.0
    #         req.torques_pt0[1] = 0.0
    #         req.torques_pt0[2] = 0.0
    #
    #         req.forces_pt1[0] = 0.0
    #         req.forces_pt1[1] = 0.0
    #         req.forces_pt1[2] = 0.0
    #
    #         req.torques_pt1[0] = 0.0
    #         req.torques_pt1[1] = 0.0
    #         req.torques_pt1[2] = 0.0
    #
    #         future = self.force_trajectory_client.call_async(req)
    #         resp = self.wait_future(future, print_waiting=False)
    #         self.get_logger().info(f"FORCE TRAJECTORY: {resp}")
    #         return resp
    #     except Exception as exc:
    #         self.get_logger().error(f"Exception: {type(exc)} - {exc}")
    #     return None

    # def hand_pose(self):
    #     try:
    #         req = HandPose.Request()
    #         future = self.hand_pose_client.call_async(req)
    #         resp = self.wait_future(future, print_waiting=False)
    #         self.get_logger().info(f"HAND POSE: {resp}")
    #         return resp
    #     except Exception as exc:
    #         self.get_logger().error(f"Exception: {type(exc)} - {exc}")
    #     return None

    def cmd_vel(self, x: float, y: float, angular: float) -> None:
        try:
            self.get_logger().info(f"PUSH VEL CMD: {x} {y} {angular}")
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.angular.z = angular
            self.cmd_vel_publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"Exception cmd_vel: {type(exc)} - {exc}")

    def body_pose(self, z: float, qx: float, qy: float, qz: float, qw: float) -> None:
        msg = Pose()
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.position.z = z
        self.body_pose_publisher.publish(msg)

    def do_test(self) -> None:
        self.power_on()
        self.stand()
        # resp = self.set_locomotion(mode=1)
        # resp = self.set_velocity(0.2, 0.2, 0.1)
        #             self.cmd_vel(0.1, 0.0, 0.0)
        time.sleep(15)
        self.sit()
        self.power_off()

    def do_command(self) -> None:
        try:
            self.timer.cancel()

            # upload_dir = "/home/tompe/spot/547_534_525.walk"
            upload_dir = "/home/tompe/spot/547_534.walk"

            if self.command == "claim":
                resp = self.claim()
                return

            if self.command == "release":
                resp = self.release()
                return

            # resp = self.claim()
            # if not resp.success:
            #    self.get_logger().error(f'CLAIM FAILED: {resp.message}')
            #    return

            if self.command == "test":
                self.do_test()

            if self.command == "power_on":
                resp = self.power_on()

            if self.command == "power_off":
                resp = self.power_off()

            if self.command == "stand":
                resp = self.stand()

            if self.command == "sit":
                resp = self.sit()

            if self.command == "start":
                resp = self.claim()
                resp = self.power_on()
                resp = self.stand()

            if self.command == "stop":
                resp = self.sit()
                # have to wait to finish
                # resp = self.release()

            self.get_logger().error(f"command: {self.command}")

            if self.command == "listgraph":
                resp = self.list_graph(upload_dir)
                self.get_logger().info(f"LISTGRAPH: {resp.waypoint_ids}")

            if self.command == "move":
                resp = self.set_locomotion(mode=2)
                # resp = self.set_velocity(0.2, 0.2, 0.1)
                for i in range(120):
                    # self.cmd_vel(0.5, 0.0, 0.0) # positive forward
                    self.cmd_vel(0.0, -0.5, 0.0)  # positive left
                    self.cmd_vel(0.5, -0.5, 0.0)
                    # self.cmd_vel(-0.5, 0.5, 0.0)

            if self.command == "trajectory":
                resp = self.set_velocity(0.5, 0.5, math.radians(30.0))
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "body"
                pose.pose.position.x = 1.0
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.0  # is not used
                # only yaw computed from quat
                yaw = 0.0
                [qx, qy, qz, qw] = quaternion_from_euler(0.0, 0.0, math.degrees(yaw))
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                duration = Duration(seconds=10, nanoseconds=0).to_msg()

                self.send_trajectory_goal(pose, duration)
                #   rclpy.spin_until_future_complete(action_client, future)

            if self.command == "navigate":
                resp = self.set_velocity(0.5, 0.5, math.radians(30.0))
                resp = self.list_graph(upload_dir)
                self.get_logger().info(f"LISTGRAPH: {resp.waypoint_ids}")
                self.send_navigate_goal(upload_dir, resp.waypoint_ids[2])
                #   rclpy.spin_until_future_complete(action_client, future)

            if self.command == "stow":
                resp = self.stow()

            if self.command == "unstow":
                resp = self.unstow()

            if self.command == "open":
                resp = self.open()

            if self.command == "close":
                resp = self.close()

            if self.command == "carry":
                resp = self.carry()

            # resp = self.release()
            # if not resp.success:
            #    self.get_logger().error(f'RELEASE FAILED: {resp.message}')
            #    return

        except Exception as exc:
            self.get_logger().error(f"EXCEPTION: {type(exc)} - {exc}")


@ros_process.main(prebaked=False)
def main() -> None:
    ros_process.spin(CommandSpotDriver, "command_spot_driver")
