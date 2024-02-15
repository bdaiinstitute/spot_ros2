#!/usr/bin/env python3
# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import time

import bdai_ros2_wrappers.process as ros_process
import bosdyn.client
import bosdyn.client.util
from bdai_ros2_wrappers.node import Node
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from cv_bridge import CvBridge
from google.protobuf.timestamp_pb2 import Timestamp
from sensor_msgs.msg import CameraInfo, Image

from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param

from .ros_helpers import bosdyn_data_to_image_and_camera_info_msgs


def translate_ros_camera_name_to_bosdyn(camera_source: str, camera_type: str) -> str:
    if camera_source == "back":
        if camera_type == "camera":
            return "back_fisheye_image"
        elif camera_type == "depth":
            return "back_depth"
        else:  # self._camera_type == "depth_registered":
            return "back_depth_in_visual_frame"
    elif camera_source == "frontleft":
        if camera_type == "camera":
            return "frontleft_fisheye_image"
        elif camera_type == "depth":
            return "frontleft_depth"
        else:  # self._camera_type == "depth_registered":
            return "frontleft_depth_in_visual_frame"
    elif camera_source == "frontright":
        if camera_type == "camera":
            return "frontright_fisheye_image"
        elif camera_type == "depth":
            return "frontright_depth"
        else:  # self._camera_type == "depth_registered":
            return "frontright_depth_in_visual_frame"
    elif camera_source == "hand":
        if camera_type == "camera":
            return "hand_color_image"
        elif camera_type == "depth":
            return "hand_depth"
        else:  # self._camera_type == "depth_registered":
            return "hand_depth_in_hand_color_frame"
    elif camera_source == "left":
        if camera_type == "camera":
            return "left_fisheye_image"
        elif camera_type == "depth":
            return "left_depth"
        else:  # self._camera_type == "depth_registered":
            return "left_depth_in_visual_frame"
    elif camera_source == "right":
        if camera_type == "camera":
            return "right_fisheye_image"
        elif camera_type == "depth":
            return "right_depth"
        else:  # self._camera_type == "depth_registered":
            return "right_depth_in_visual_frame"

    raise ValueError(f"Invalid camera source {camera_source} or camera_type {camera_type} supplied")


class SpotImagePublisher(Node):
    def __init__(self) -> None:
        super().__init__("spot_image_publisher")
        self.declare_parameter("spot_name", "")
        self._spot_name = self.get_parameter("spot_name").value

        self._username = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_USERNAME", self, "username", "user")
        self._password = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_PASSWORD", self, "password", "password")
        self._robot_hostname = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")

        self.declare_parameter("image_service", ImageClient.default_service_name)
        self._image_service = self.get_parameter("image_service").value

        self.declare_parameter("rgb_cameras", True)
        self._rgb_cameras = self.get_parameter("rgb_cameras").value

        self._cv_bridge = CvBridge()

        try:
            self._sdk = bosdyn.client.create_standard_sdk("image_publisher")
        except Exception as e:
            self._logger.error("Error creating SDK object: %s", e)
            self._valid = False
            return

        # Create robot object with an image client.
        self._robot = self._sdk.create_robot(self._robot_hostname)

        try:
            self._robot.authenticate(self._username, self._password)
            self._robot.start_time_sync()
        except bosdyn.client.RpcError as err:
            self._logger.error("Failed to communicate with robot: %s", err)
            self._valid = False
            return

        sdk = bosdyn.client.create_standard_sdk("image_capture")
        robot = sdk.create_robot(self._robot_hostname)
        bosdyn.client.util.authenticate(robot)
        robot.sync_with_directory()
        robot.time_sync.wait_for_sync()
        self._image_client = robot.ensure_client(self._image_service)

        self.declare_parameter("image_publish_rate", 10)
        self._image_publish_rate = self.get_parameter("image_publish_rate").value

        self.declare_parameter("camera_type", "")
        self._camera_type = self.get_parameter("camera_type").value
        if self._camera_type not in ["camera", "depth", "depth_registered"]:
            raise ValueError(
                'camera_source must be in ["camera", "depth", "depth_registered"], '
                f"received {self._camera_type} instead"
            )

        self._frame_prefix = ""
        if self._spot_name != "":
            self._frame_prefix = f"{self._spot_name}/"

        self._image_requests = []
        self._image_publishers = {}
        self._camera_info_publishers = {}
        camera_sources = ["frontleft", "frontright", "left", "right", "back"]
        if robot.has_arm():
            camera_sources.append("hand")
        for camera_source in camera_sources:
            if self._camera_type == "camera":
                if self._rgb_cameras:
                    pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
                else:
                    pixel_format = image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
            else:
                pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
            bosdyn_camera_source = translate_ros_camera_name_to_bosdyn(camera_source, self._camera_type)
            image_request = build_image_request(bosdyn_camera_source, pixel_format=pixel_format)
            print(bosdyn_camera_source)
            self._image_requests.append(image_request)

            self._camera_info_publishers[bosdyn_camera_source] = self.create_publisher(
                CameraInfo,
                f"{self._camera_type}/{camera_source}/camera_info",
                10,
            )

            self._image_publishers[bosdyn_camera_source] = self.create_publisher(
                Image,
                f"{self._camera_type}/{camera_source}/image",
                10,
            )

        self._image_publisher_timer = self.create_timer(1 / self._image_publish_rate, self.publish_image)

    def robot_to_local_time(self, timestamp: Timestamp) -> Timestamp:
        """Takes a timestamp and an estimated skew and return seconds and nanoseconds in local time

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """

        rtime = Timestamp()

        time_skew = self._robot.time_sync.endpoint.clock_skew
        rtime.seconds = timestamp.seconds - time_skew.seconds
        rtime.nanos = timestamp.nanos - time_skew.nanos
        if rtime.nanos < 0:
            rtime.nanos = rtime.nanos + 1000000000
            rtime.seconds = rtime.seconds - 1

        # Workaround for timestamps being incomplete
        if rtime.seconds < 0:
            rtime.seconds = 0
            rtime.nanos = 0

        return rtime

    def publish_image(self) -> None:
        start_time = time.time()
        image_responses = self._image_client.get_image(self._image_requests)
        time1 = time.time()
        # print(f"Time taken to get responses is {time1-start_time}s")
        if len(image_responses) == 0:
            self.get_logger().info("No images received")
            return

        for image_response in image_responses:
            image_msg, camera_info_msg = bosdyn_data_to_image_and_camera_info_msgs(
                image_response, self.robot_to_local_time, self._frame_prefix
            )
            self._image_publishers[image_response.source.name].publish(image_msg)
            self._camera_info_publishers[image_response.source.name].publish(camera_info_msg)
        time2 = time.time()
        print(f"Time taken to publish responses is {time2-time1}s")
        print(f"Overall time taken is {time2-start_time}")


@ros_process.main(prebaked=False)
def main() -> None:
    ros_process.spin(SpotImagePublisher)


if __name__ == "__main__":
    main()
