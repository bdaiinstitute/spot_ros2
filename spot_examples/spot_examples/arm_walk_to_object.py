import argparse
from typing import Optional
import time
import os

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import bosdyn.geometry
from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from sensor_msgs.msg import Image
from spot_msgs.action import RobotCommand, Manipulation

from cv_bridge import CvBridge
import cv2

from .simple_spot_commander import SimpleSpotCommander

from sensor_msgs.msg import CameraInfo

class ArmWalkToObject:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self.node = ros_scope.node()
        if self.node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self.logger = self.node.get_logger()

        self.image_sub = self.node.create_subscription(
            Image,
            f"/{robot_name}/camera/frontleft/image",
            self.image_callback,
            10)
        self.image_sub # prevent unused variable warning
        self.pause_image_update = False

        self.br = CvBridge()

        self.image_click = None
        self.image_display = None


        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.tf_listener = TFListenerWrapper(node)
        self.tf_listener.wait_for_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        self.robot = SimpleSpotCommander(robot_name, node)
        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)

        self.manipulation_client = ActionClientWrapper(
            Manipulation, namespace_with(robot_name, "manipulation"), node)


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

    def stand(self) -> bool:
        self.logger.info("Standing robot up")
        result = self.robot.command("stand")
        if not result.success:
            self.logger.error("Robot did not stand message was " + result.message)
            return False
        self.logger.info("Successfully stood up.")
        time.sleep(3)
        return True

    # def take_photo(self) -> None:
    #     footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
    #
    #     cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    #     action_goal = RobotCommand.Goal()
    #     convert(cmd, action_goal.command)
    #     self.logger.info("Twisting robot")
    #     self.robot_command_client.send_goal_and_wait("twisting_robot", action_goal)
    #
    #     self.logger.info('Robot standing twisted.')
    #     time.sleep(3)

    def image_callback(self, image_raw):
        self.logger.debug("recieved image")
        self.latest_image_raw = image_raw

    def query_user_and_go_to_object(self, display_time=3.0):




        # Show the image to the user and wait for them to click on a pixel
        self.logger.info('Click on an object to walk up to...')
        image_title = 'Click to walk up to something'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, self.cv_mouse_callback)

        self.image_display = self.br.imgmsg_to_cv2(self.latest_image_raw)
        cv2.imshow(image_title, self.image_display)
        while self.image_click is None:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                print('"q" pressed, exiting.')
                exit(0)


        self.logger.info(f"Walking to object at image location ({self.image_click[0]}, {self.image_click[1]})")

        walk_vec = geometry_pb2.Vec2(x=self.image_click[0], y=self.image_click[1])

        # Optionally populate the offset distance parameter.
        if config.distance is None:
            offset_distance = None
        else:
            offset_distance = wrappers_pb2.FloatValue(value=config.distance)

        # Build the proto
        walk_to = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=walk_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole, offset_distance=offset_distance)

        # Ask the robot to pick up the object
        walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
            walk_to_object_in_image=walk_to)


        ## Manipulation.msg: 
        # bosdyn_api_msgs/ManipulationApiRequest command
        # ---
        # bosdyn_api_msgs/ManipulationApiFeedbackResponse result
        # bool success
        # string message
        # ---
        # bosdyn_api_msgs/ManipulationApiFeedbackResponse feedback

        # cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)

        action_goal = Manipulation.Goal()
        # convert(cmd, action_goal.command)
        action_goal.command = walk_to_request
        self.logger.info("Walking to object in image")
        self.manipulation_client.send_goal_and_wait("walking to object in image", action_goal)
        self.logger.info("Finished walking to object in image.")


        ## TODO: implement action goal feedback, probably with send_goal_and_wait_handle

        ## other candidiate
        # goal = Manipulation.Goal()
        # command_builder = SpotManipulationCommandBuilder(self)
        # command = command_builder.walk_to_object_in_image_command(
        #     image_pixel_xy=pixel_xy, image_timestamp=image_timestamp, camera_info=camera_data, offset_distance=offset
        # )
        # convert(command, goal.command)
        # return self.manipulation("walk_to_object_image", goal, timeout_sec=timeout_sec)


    def cv_mouse_callback(self, event, x, y, flags, param):
        clone = self.image_display.copy()
        if event == cv2.EVENT_LBUTTONUP:
            self.image_click = (x, y)
        else:
            # Draw some lines on the image.
            #print('mouse', x, y)
            color = (30, 30, 30)
            thickness = 2
            image_title = 'Click to walk up to something'
            height = clone.shape[0]
            width = clone.shape[1]
            cv2.line(clone, (0, y), (width, y), color, thickness)
            cv2.line(clone, (x, 0), (x, height), color, thickness)
            cv2.imshow(image_title, clone)


def arg_float(x):
    try:
        x = float(x)
    except ValueError:
        raise argparse.ArgumentTypeError(f'{repr(x)} not a number')
    return x


## example from hello_spot.py
# def cli() -> argparse.ArgumentParser:
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--robot", type=str, default=None)
#     return parser
#
#
# @ros_process.main(cli())
# def main(args: argparse.Namespace) -> None:
#     hello_spot = HelloSpot(args.robot)
#     hello_spot.initialize_robot()
#     hello_spot.stand_default()
#     hello_spot.stand_twisted()
#     hello_spot.stand_3_pt_traj()
#

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    # parser.add_argument('-i', '--image-source', help='Get image from source',
    #                     default='frontleft_fisheye_image')
    # parser.add_argument('-d', '--distance', help='Distance from object to walk to (meters).',
    #                     default=None, type=arg_float)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_spot = ArmWalkToObject(args.robot)
    hello_spot.initialize_robot()
    hello_spot.stand()
    # hello_spot.take_photo()
    hello_spot.query_user_and_go_to_object()



if __name__ == "__main__":
    main()
