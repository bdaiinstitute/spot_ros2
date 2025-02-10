import argparse
import os
import time
from typing import Optional

import bosdyn.geometry
import cv2
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration
from bosdyn_msgs.conversions import convert
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander


class HelloSpot:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self.node = ros_scope.node()
        if self.node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self.logger = self.node.get_logger()

        self.image_sub = self.node.create_subscription(
            Image, namespace_with(robot_name, "camera/frontleft/image"), self.image_callback, 10
        )
        self.image_sub  # prevent unused variable warning
        self.pause_image_update = False

        self.br = CvBridge()

        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.tf_listener = TFListenerWrapper(node)
        self.tf_listener.wait_for_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        self.robot = SimpleSpotCommander(robot_name, node)
        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)

    def initialize_robot(self) -> bool:
        # Claim robot
        self.logger.info("Claiming robot")
        result = self.robot.command("claim")
        if not result.success:
            self.node.get_logger().error("Unable to claim robot message was " + result.message)
            return False
        self.logger.info("Claimed robot")

        # Power on robot
        self.logger.info("Powering on robot")
        result = self.robot.command("power_on")
        if not result.success:
            self.logger.error("Unable to power on robot message was " + result.message)
            return False
        return True

    def stand_default(self) -> bool:
        self.logger.info("Standing robot up")
        result = self.robot.command("stand")
        if not result.success:
            self.logger.error("Robot did not stand message was " + result.message)
            return False
        self.logger.info("Successfully stood up.")
        time.sleep(3)
        return True

    def stand_twisted(self) -> None:
        # Tell the robot to stand in a twisted position.
        #
        # The RobotCommandBuilder constructs command messages, which are then
        # issued to the robot using "robot_command" on the command client.
        #
        # In this example, the RobotCommandBuilder generates a stand command
        # message with a non-default rotation in the footprint frame. The footprint
        # frame is a gravity aligned frame with its origin located at the geometric
        # center of the feet. The X axis of the footprint frame points forward along
        # the robot's length, the Z axis points up aligned with gravity, and the Y
        # axis is the cross-product of the two.
        footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)

        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        action_goal = RobotCommand.Goal()
        convert(cmd, action_goal.command)
        self.logger.info("Twisting robot")
        self.robot_command_client.send_goal_and_wait("twisting_robot", action_goal)

        self.logger.info("Robot standing twisted.")
        time.sleep(3)

    def stand_3_pt_traj(self) -> None:
        # Now compute an absolute desired position and orientation of the robot body origin.
        # Use the frame helper class to compute the world to gravity aligned body frame transformation.
        # Note, the robot_state used here was cached from before the above yaw stand command,
        # so it contains the nominal stand pose.
        # odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
        #                                  ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        ## spot_ros2 replacement
        odom_T_flat_body = self.tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)

        odom_T_flat_body_se3 = math_helpers.SE3Pose(
            odom_T_flat_body.transform.translation.x,
            odom_T_flat_body.transform.translation.y,
            odom_T_flat_body.transform.translation.z,
            math_helpers.Quat(
                odom_T_flat_body.transform.rotation.w,
                odom_T_flat_body.transform.rotation.x,
                odom_T_flat_body.transform.rotation.y,
                odom_T_flat_body.transform.rotation.z,
            ),
        )

        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        t1 = 2.5
        t2 = 5.0
        t3 = 7.5

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0.075, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat(w=0.9848, x=0, y=0.1736, z=0))
        flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat())

        # Build the points in the trajectory.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body_se3 * flat_body_T_pose1).to_proto(), time_since_reference=seconds_to_duration(t1)
        )
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body_se3 * flat_body_T_pose2).to_proto(), time_since_reference=seconds_to_duration(t2)
        )
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body_se3 * flat_body_T_pose3).to_proto(), time_since_reference=seconds_to_duration(t3)
        )

        # Build the trajectory proto by combining the points.
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2, traj_point3])

        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(
                root_frame_name=ODOM_FRAME_NAME, base_offset_rt_root=traj
            )
        )

        mobility_params = spot_command_pb2.MobilityParams(body_control=body_control)

        stand_command = RobotCommandBuilder.synchro_stand_command(params=mobility_params)

        stand_command_goal = RobotCommand.Goal()
        convert(stand_command, stand_command_goal.command)
        self.logger.info("Beginning absolute body control while standing.")
        self.robot_command_client.send_goal_and_wait(action_name="hello_spot", goal=stand_command_goal, timeout_sec=10)
        self.logger.info("Finished absolute body control while standing.")

        self.logger.info("Displaying image.")
        self._maybe_display_image()
        self._maybe_save_image()

        self.logger.info("Goodbye, human!")

    def image_callback(self, image_raw: Image) -> None:
        self.logger.debug("recieved image")
        if self.pause_image_update:
            self.logger.debug("pausing image update while displaying and saving current image.")
            return
        self.latest_image_raw = image_raw

    def _maybe_display_image(self, display_time: float = 3.0) -> None:
        """Try to display image, if client has correct deps."""

        self.pause_image_update = True  # to ensure we display and save same image

        try:
            image = self.br.imgmsg_to_cv2(self.latest_image_raw)
            cv2.imshow("Hello, human!", image)
            cv2.waitKey(0)
            time.sleep(display_time)
        except Exception as exc:
            self.logger.warning("Exception thrown displaying image. %r", exc)

    def _maybe_save_image(self, path: Optional[str] = None) -> None:
        """Try to save image, if client has correct deps."""

        name = "hello-spot-img.jpg"
        if path is not None and os.path.exists(path):
            path = os.path.join(os.getcwd(), path)
            name = os.path.join(path, name)
            self.logger.info("Saving image to: %s", name)
        else:
            self.logger.info(f"Saving image to working directory as {name}")
        try:
            image = self.br.imgmsg_to_cv2(self.latest_image_raw)
            cv2.imwrite(name, image)

        except Exception as exc:
            self.logger.warning("Exception thrown saving image. %r", exc)

        self.pause_image_update = False  # resume image updating


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_spot = HelloSpot(args.robot)
    hello_spot.initialize_robot()
    hello_spot.stand_default()
    hello_spot.stand_twisted()
    hello_spot.stand_3_pt_traj()


if __name__ == "__main__":
    main()
