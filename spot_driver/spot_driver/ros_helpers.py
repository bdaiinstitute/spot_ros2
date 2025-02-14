import os
import time
from typing import Any, Callable, List, Optional, Tuple, Union

import builtin_interfaces.msg
import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_py as tf2
import tf2_ros
from bosdyn.api import image_pb2, world_object_pb2
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.math_helpers import SE3Pose
from bosdyn_api_msgs.math_helpers import ros_transform_to_se3_pose
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from google.protobuf.timestamp_pb2 import Timestamp
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from spot_wrapper.wrapper import SpotWrapper

cv_bridge = CvBridge()


def populate_transform_stamped(
    tf_time: builtin_interfaces.msg.Time, parent_frame: str, child_frame: str, transform: SE3Pose, frame_prefix: str
) -> TransformStamped:
    """Populates a TransformStamped message
    Args:
        frame_prefix: The prefix to add to the frames
        tf_time: The time of the transform
        parent_frame: The parent frame of the transform
        child_frame: The child_frame_id of the transform
        transform: A transform to copy into a StampedTransform object. Should have position (x,y,z) and rotation (x,
        y,z,w) members
    Returns:
        TransformStamped message
    """
    new_tf = TransformStamped()
    new_tf.header.stamp = tf_time
    new_tf.header.frame_id = parent_frame
    if "/" not in parent_frame:
        new_tf.header.frame_id = frame_prefix + parent_frame
    new_tf.child_frame_id = child_frame
    if "/" not in child_frame:
        new_tf.child_frame_id = frame_prefix + child_frame
    try:
        [x, y, z, rx, ry, rz, rw] = [
            transform.position.x,
            transform.position.y,
            transform.position.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
    except AttributeError:
        [x, y, z, rx, ry, rz, rw] = [
            transform.x,
            transform.y,
            transform.z,
            transform.rot.x,
            transform.rot.y,
            transform.rot.z,
            transform.rot.w,
        ]
    new_tf.transform.translation.x = x
    new_tf.transform.translation.y = y
    new_tf.transform.translation.z = z
    new_tf.transform.rotation.x = rx
    new_tf.transform.rotation.y = ry
    new_tf.transform.rotation.z = rz
    new_tf.transform.rotation.w = rw

    return new_tf


def create_default_camera_info() -> CameraInfo:
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

    return camera_info_msg


def _create_compressed_image_msg(
    data: image_pb2.ImageResponse, robot_to_local_time: Callable[[Timestamp], Timestamp], frame_prefix: str
) -> CompressedImage:
    image_msg = CompressedImage()
    local_time = robot_to_local_time(data.shot.acquisition_time)
    image_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    image_msg.header.frame_id = frame_prefix + data.shot.frame_name_image_sensor
    image_msg.format = "jpeg"
    image_msg.data = data.shot.image.data
    return image_msg


def _create_image_msg(
    data: image_pb2.ImageResponse, robot_to_local_time: Callable[[Timestamp], Timestamp], frame_prefix: str
) -> Image:
    image_msg = None
    local_time = robot_to_local_time(data.shot.acquisition_time)
    stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    frame_id = frame_prefix + data.shot.frame_name_image_sensor

    # JPEG format
    if data.shot.image.format == image_pb2.Image.FORMAT_JPEG:
        cv2_image = cv2.imdecode(np.frombuffer(data.shot.image.data, dtype=np.uint8), -1)
        image_msg = cv_bridge.cv2_to_imgmsg(cv2_image, encoding="passthrough")
        if image_msg.encoding == "8UC3":
            image_msg.encoding = "bgr8"  # required for cv_bridge handling of the message
        elif image_msg.encoding == "8UC1":
            image_msg.encoding = "mono8"  # required for cv_bridge handling of the message
        else:
            pass  # passthrough decides
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = frame_id

    # Uncompressed.  Requires pixel_format.
    elif data.shot.image.format == image_pb2.Image.FORMAT_RAW:
        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = frame_id
        image_msg.height = data.shot.image.rows
        image_msg.width = data.shot.image.cols

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
    return image_msg


def bosdyn_data_to_image_and_camera_info_msgs(
    data: image_pb2.ImageResponse, robot_to_local_time: Callable[[Timestamp], Timestamp], frame_prefix: str
) -> Tuple[Union[Image, CompressedImage], CameraInfo]:
    """Takes the image and camera data and populates the necessary ROS messages
    Args:
        data: Image proto
        robot_to_local_time: Function to convert the robot time to the local time
        frame_prefix: namespace for the published images

    Returns:
        (tuple):
            * Image: message of the image captured
            * CameraInfo: message to define the state and config of the camera that took the image
    """
    image_msg = _create_image_msg(data, robot_to_local_time, frame_prefix)

    # camera_info_msg = create_default_camera_info(camera_info_msg)
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
    local_time = robot_to_local_time(data.shot.acquisition_time)
    camera_info_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    camera_info_msg.header.frame_id = frame_prefix + data.shot.frame_name_image_sensor
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


def get_frame_names_associated_with_object(world_object: world_object_pb2.WorldObject) -> List[str]:
    possible_frame_names = [
        world_object.apriltag_properties.frame_name_fiducial,
        world_object.apriltag_properties.frame_name_fiducial_filtered,
        world_object.dock_properties.frame_name_dock,
        world_object.image_properties.frame_name_image_coordinates,
    ]
    frame_names = [name for name in possible_frame_names if name]
    for drawable in world_object.drawable_properties:
        frame_names.append(drawable.frame_name_drawable)

    return frame_names


def get_tf_from_world_objects(
    world_objects: List[world_object_pb2.WorldObject], spot_wrapper: SpotWrapper, parent_frame: str
) -> TFMessage:
    tf_msg = TFMessage()
    for world_object in world_objects:
        frames = get_frame_names_associated_with_object(world_object)
        for frame in frames:
            try:
                spot_parent_frame = parent_frame[parent_frame.rfind("/") + 1 :]
                transform = get_a_tform_b(world_object.transforms_snapshot, spot_parent_frame, frame)
                local_time = spot_wrapper.robotToLocalTime(world_object.acquisition_time)
                tf_time = Time(sec=local_time.seconds, nanosec=local_time.nanos)
                new_tf = populate_transform_stamped(tf_time, parent_frame, frame, transform, spot_wrapper.frame_prefix)
                tf_msg.transforms.append(new_tf)
            except Exception as e:
                spot_wrapper.logger.error("Error: {}".format(e))
    return tf_msg


def get_from_env_and_fall_back_to_param(env_name: str, node: Node, param_name: str, default_value: Any) -> Any:
    value = os.environ.get(env_name)
    if value is None:
        node.declare_parameter(param_name, default_value)
        value = node.get_parameter(param_name).value
    value_type = type(default_value)
    return value_type(value)


# Timeout only works if your tf listener updates in a separate thread!
def lookup_a_tform_b(
    tf_buffer: tf2_ros.Buffer,
    frame_a: str,
    frame_b: str,
    transform_time: Optional[rclpy.time.Time] = None,
    timeout: Optional[float] = None,
    wait_for_frames: bool = False,
) -> Optional[SE3Pose]:
    if transform_time is None:
        transform_time = rclpy.time.Time()
    if timeout is None or not wait_for_frames:
        timeout_py = rclpy.time.Duration()
    else:
        timeout_py = rclpy.time.Duration(seconds=timeout)
    start_time = time.time()
    while rclpy.ok():
        try:
            return ros_transform_to_se3_pose(
                tf_buffer.lookup_transform(frame_a, frame_b, time=transform_time, timeout=timeout_py).transform
            )
        except tf2.ExtrapolationException as e:
            if "future" not in str(e):
                raise e  # Waiting won't help with this
            now = time.time()
            if timeout is None or now - start_time > timeout:
                raise e
            time.sleep(0.01)
    return None


class TriggerServiceWrapper:  # TODO: caching of spot_ros might be funky with multiple instances up
    """A descriptor for calling callbacks for trigger services"""

    service_func: Callable[[SpotWrapper], Trigger.Response]
    service_name: str
    args: Optional[tuple] = ()
    kwargs: Optional[dict | None] = None

    def __init__(
        self, service_func: Callable, service_name: str, *args: Optional[Any], **kwargs: Optional[Any | None]
    ) -> None:
        self.service_func = service_func
        self.service_name = service_name
        self.args = args
        self.kwargs = kwargs or {}

    def create_service(self, obj: Any, group: CallbackGroup) -> None:
        self.spot_ros = obj
        obj.create_service(Trigger, self.service_name, self.callback, callback_group=group)

    def callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.spot_ros.mock:
            response.success = True
            response.message = "Mock spot success"
            return response

        if self.spot_ros.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response

        try:
            # Call the function with predefined arguments
            response.success, response.message = self.service_func(
                self.spot_ros.spot_wrapper, *self.args, **self.kwargs
            )  # type: ignore
        except Exception as e:
            response.success = False
            response.message = f"Error executing {self.service_func}: {str(e)}"

        return response
