# Copyright [2023] Boston Dynamics AI Institute, Inc.

import os
import sys
import time

import rclpy
from sensor_msgs.msg import CameraInfo, Image
import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from google.protobuf.timestamp_pb2 import Timestamp
from builtin_interfaces.msg import Time, Duration
from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param
import rclpy.node


def translate_ros_camera_name_to_bosdyn(camera_source: str, camera_type: str):
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


class SpotImagePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__(f"spot_image_publisher")
        self.declare_parameter('spot_name', '')
        self._spot_name = self.get_parameter('spot_name').value

        self._username = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_USERNAME", self, "username", "user")
        self._password = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_PASSWORD", self, "password", "password")
        self._robot_hostname = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")

        self.declare_parameter("image_service", ImageClient.default_service_name)
        self._image_service = self.get_parameter("image_service").value

        try:
            self._sdk = bosdyn.client.create_standard_sdk('image_publisher')
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

        sdk = bosdyn.client.create_standard_sdk('image_capture')
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
            raise ValueError(f"camera_source must be in [\"camera\", \"depth\", \"depth_registered\"], "
                             f"received {self._camera_type} instead")

        self._frame_prefix = ''
        if self._spot_name != "":
            self._frame_prefix = f"{self._spot_name}/"

        # TODO: Add Hand
        self._image_requests = []
        self._image_publishers = {}
        self._camera_info_publishers = {}
        camera_sources = ["frontleft", "frontright", "left", "right", "back"]
        for camera_source in camera_sources:
            if self._camera_type == "camera":
                pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
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

        self._image_publisher_timer = self.create_timer(1/self._image_publish_rate, self.publish_image)

    def robotToLocalTime(self, timestamp):
        """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

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

    def bosdyn_data_to_image_and_camera_info_msgs(self, data):
        """Takes the image and camera data and populates the necessary ROS messages
        Args:
            data: Image proto
        Returns:
            (tuple):
                * Image: message of the image captured
                * CameraInfo: message to define the state and config of the camera that took the image
        """
        image_msg = Image()
        local_time = self.robotToLocalTime(data.shot.acquisition_time)
        image_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
        image_msg.header.frame_id = self._frame_prefix + data.shot.frame_name_image_sensor
        image_msg.height = data.shot.image.rows
        image_msg.width = data.shot.image.cols

        # Color/greyscale formats.
        # JPEG format
        if data.shot.image.format == image_pb2.Image.FORMAT_JPEG:
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = True
            image_msg.step = 3 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Uncompressed.  Requires pixel_format.
        if data.shot.image.format == image_pb2.Image.FORMAT_RAW:
            # One byte per pixel.
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                image_msg.encoding = "mono8"
                image_msg.is_bigendian = True
                image_msg.step = data.shot.image.cols
                image_msg.data = data.shot.image.data

            # Three bytes per pixel.
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                image_msg.encoding = "rgb8"
                image_msg.is_bigendian = True
                image_msg.step = 3 * data.shot.image.cols
                image_msg.data = data.shot.image.data

            # Four bytes per pixel.
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                image_msg.encoding = "rgba8"
                image_msg.is_bigendian = True
                image_msg.step = 4 * data.shot.image.cols
                image_msg.data = data.shot.image.data

            # Little-endian uint16 z-distance from camera (mm).
            if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                image_msg.encoding = "16UC1"
                image_msg.is_bigendian = False
                image_msg.step = 2 * data.shot.image.cols
                image_msg.data = data.shot.image.data

        # camera_info_msg = createDefaulCameraInfo(camera_info_msg)
        camera_info_msg = CameraInfo()
        camera_info_msg.distortion_model = "plumb_bob"

        camera_info_msg.d.append(0)
        camera_info_msg.d.append(0)
        camera_info_msg.d.append(0)
        camera_info_msg.d.append(0)
        camera_info_msg.d.append(0)

        camera_info_msg.k[1] = 0
        camera_info_msg.k[3] = 0
        camera_info_msg.k[6] = 0
        camera_info_msg.k[7] = 0
        camera_info_msg.k[8] = 1

        camera_info_msg.r[0] = 1
        camera_info_msg.r[1] = 0
        camera_info_msg.r[2] = 0
        camera_info_msg.r[3] = 0
        camera_info_msg.r[4] = 1
        camera_info_msg.r[5] = 0
        camera_info_msg.r[6] = 0
        camera_info_msg.r[7] = 0
        camera_info_msg.r[8] = 1

        camera_info_msg.p[1] = 0
        camera_info_msg.p[3] = 0
        camera_info_msg.p[4] = 0
        camera_info_msg.p[7] = 0
        camera_info_msg.p[8] = 0
        camera_info_msg.p[9] = 0
        camera_info_msg.p[10] = 1
        camera_info_msg.p[11] = 0
        local_time = self.robotToLocalTime(data.shot.acquisition_time)
        camera_info_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
        camera_info_msg.header.frame_id = data.shot.frame_name_image_sensor
        camera_info_msg.height = data.shot.image.rows
        camera_info_msg.width = data.shot.image.cols

        camera_info_msg.k[0] = data.source.pinhole.intrinsics.focal_length.x
        camera_info_msg.k[2] = data.source.pinhole.intrinsics.principal_point.x
        camera_info_msg.k[4] = data.source.pinhole.intrinsics.focal_length.y
        camera_info_msg.k[5] = data.source.pinhole.intrinsics.principal_point.y

        camera_info_msg.p[0] = data.source.pinhole.intrinsics.focal_length.x
        camera_info_msg.p[2] = data.source.pinhole.intrinsics.principal_point.x
        camera_info_msg.p[5] = data.source.pinhole.intrinsics.focal_length.y
        camera_info_msg.p[6] = data.source.pinhole.intrinsics.principal_point.y

        return image_msg, camera_info_msg

    def publish_image(self):
        start_time = time.time()
        image_responses = self._image_client.get_image(self._image_requests)
        time1 = time.time()
        # print(f"Time taken to get responses is {time1-start_time}s")
        if len(image_responses) == 0:
            self.get_logger().info("No images received")
            return

        for image_response in image_responses:
            image_msg, camera_info_msg = self.bosdyn_data_to_image_and_camera_info_msgs(image_response)
            self._image_publishers[image_response.source.name].publish(image_msg)
            self._camera_info_publishers[image_response.source.name].publish(camera_info_msg)
        time2 = time.time()
        print(f"Time taken to publish responses is {time2-time1}s")
        print(f"Overall time taken is {time2-start_time}")


def main() -> None:
    rclpy.init(args=sys.argv)
    spot_image_publisher = SpotImagePublisher()
    rclpy.spin(spot_image_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    exit(main())
