import os
import time
from typing import Callable, List, Optional, Tuple, Union

import builtin_interfaces.msg
import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_py as tf2
import tf2_ros
from bosdyn.api import image_pb2, robot_state_pb2, world_object_pb2
from bosdyn.client.frame_helpers import get_a_tform_b, get_odom_tform_body, get_vision_tform_body
from bosdyn.client.math_helpers import SE3Pose
from bosdyn_msgs.msg import ManipulatorState
from builtin_interfaces.msg import Duration, Time
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    PoseWithCovariance,
    TransformStamped,
    TwistWithCovarianceStamped,
    Vector3Stamped,
)
from google.protobuf.timestamp_pb2 import Timestamp
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from tf2_msgs.msg import TFMessage

from spot_driver.conversions import convert_proto_to_bosdyn_msgs_manipulator_state
from spot_msgs.msg import (  # type: ignore
    BatteryState,
    BatteryStateArray,
    BehaviorFault,
    BehaviorFaultState,
    EStopState,
    EStopStateArray,
    FootState,
    FootStateArray,
    PowerState,
    SystemFault,
    SystemFaultState,
    WiFiState,
)
from spot_wrapper.wrapper import SpotWrapper

try:
    from conversions import ros_transform_to_se3_pose
except ModuleNotFoundError:
    from .manual_conversions import ros_transform_to_se3_pose


friendly_joint_names = {}
"""Dictionary for mapping BD joint names to more friendly names"""
friendly_joint_names["fl.hx"] = "front_left_hip_x"
friendly_joint_names["fl.hy"] = "front_left_hip_y"
friendly_joint_names["fl.kn"] = "front_left_knee"
friendly_joint_names["fr.hx"] = "front_right_hip_x"
friendly_joint_names["fr.hy"] = "front_right_hip_y"
friendly_joint_names["fr.kn"] = "front_right_knee"
friendly_joint_names["hl.hx"] = "rear_left_hip_x"
friendly_joint_names["hl.hy"] = "rear_left_hip_y"
friendly_joint_names["hl.kn"] = "rear_left_knee"
friendly_joint_names["hr.hx"] = "rear_right_hip_x"
friendly_joint_names["hr.hy"] = "rear_right_hip_y"
friendly_joint_names["hr.kn"] = "rear_right_knee"
friendly_joint_names["arm0.sh0"] = "arm_sh0"
friendly_joint_names["arm0.sh1"] = "arm_sh1"
friendly_joint_names["arm0.hr0"] = "arm_hr0"
friendly_joint_names["arm0.el0"] = "arm_el0"
friendly_joint_names["arm0.el1"] = "arm_el1"
friendly_joint_names["arm0.wr0"] = "arm_wr0"
friendly_joint_names["arm0.wr1"] = "arm_wr1"
friendly_joint_names["arm0.f1x"] = "arm_f1x"
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


def get_joint_states_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> JointState:
    """Maps joint state data from robot state proto to ROS JointState message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        JointState message
    """
    joint_state = JointState()
    local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
    joint_state.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    for joint in state.kinematic_state.joint_states:
        joint_state.name.append(spot_wrapper.frame_prefix + friendly_joint_names.get(joint.name, "ERROR"))
        joint_state.position.append(joint.position.value)
        joint_state.velocity.append(joint.velocity.value)
        joint_state.effort.append(joint.load.value)

    return joint_state


def get_manipulator_state_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> ManipulatorState:
    """Maps manipulation state data from robot state proto to Bosdyn ManipulatorState message

    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object

    Returns:
        ManipulatorState message
    """
    manipulator_state = ManipulatorState()
    convert_proto_to_bosdyn_msgs_manipulator_state(proto=state.manipulator_state, ros_msg=manipulator_state)
    return manipulator_state


def get_estop_state_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> EStopStateArray:
    """Maps eStop state data from robot state proto to ROS EStopArray message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        EStopArray message
    """
    estop_array_msg = EStopStateArray()
    for estop in state.estop_states:
        estop_msg = EStopState()
        local_time = spot_wrapper.robotToLocalTime(estop.timestamp)
        estop_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
        estop_msg.name = estop.name
        estop_msg.type = estop.type
        estop_msg.state = estop.state
        estop_msg.state_description = estop.state_description
        estop_array_msg.estop_states.append(estop_msg)

    return estop_array_msg


def get_feet_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> FootStateArray:
    """Maps foot position state data from robot state proto to ROS FootStateArray message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        FootStateArray message
    """
    foot_array_msg = FootStateArray()
    for foot in state.foot_state:
        foot_msg = FootState()
        foot_msg.foot_position_rt_body.x = foot.foot_position_rt_body.x
        foot_msg.foot_position_rt_body.y = foot.foot_position_rt_body.y
        foot_msg.foot_position_rt_body.z = foot.foot_position_rt_body.z
        foot_msg.contact = foot.contact
        foot_array_msg.states.append(foot_msg)

    return foot_array_msg


def get_odom_twist_from_state(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper
) -> TwistWithCovarianceStamped:
    """Maps odometry data from robot state proto to ROS TwistWithCovarianceStamped message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        TwistWithCovarianceStamped message
    """
    twist_odom_msg = TwistWithCovarianceStamped()
    local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
    twist_odom_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    twist_odom_msg.twist.twist.linear.x = state.kinematic_state.velocity_of_body_in_odom.linear.x
    twist_odom_msg.twist.twist.linear.y = state.kinematic_state.velocity_of_body_in_odom.linear.y
    twist_odom_msg.twist.twist.linear.z = state.kinematic_state.velocity_of_body_in_odom.linear.z
    twist_odom_msg.twist.twist.angular.x = state.kinematic_state.velocity_of_body_in_odom.angular.x
    twist_odom_msg.twist.twist.angular.y = state.kinematic_state.velocity_of_body_in_odom.angular.y
    twist_odom_msg.twist.twist.angular.z = state.kinematic_state.velocity_of_body_in_odom.angular.z
    return twist_odom_msg


def get_odom_from_state(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper, use_vision: bool = True
) -> Odometry:
    """Maps odometry data from robot state proto to ROS Odometry message
    Args:
        use_vision: Add vision frame, else add odom frame
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        Odometry message
    """
    odom_msg = Odometry()
    local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
    odom_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    if use_vision is True:
        odom_msg.header.frame_id = spot_wrapper.frame_prefix + "vision"
        tform_body = get_vision_tform_body(state.kinematic_state.transforms_snapshot)
    else:
        odom_msg.header.frame_id = spot_wrapper.frame_prefix + "odom"
        tform_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
    odom_msg.child_frame_id = spot_wrapper.frame_prefix + "body"
    pose_odom_msg = PoseWithCovariance()
    pose_odom_msg.pose.position.x = tform_body.position.x
    pose_odom_msg.pose.position.y = tform_body.position.y
    pose_odom_msg.pose.position.z = tform_body.position.z
    pose_odom_msg.pose.orientation.x = tform_body.rotation.x
    pose_odom_msg.pose.orientation.y = tform_body.rotation.y
    pose_odom_msg.pose.orientation.z = tform_body.rotation.z
    pose_odom_msg.pose.orientation.w = tform_body.rotation.w

    odom_msg.pose = pose_odom_msg
    twist_odom_msg = get_odom_twist_from_state(state, spot_wrapper).twist
    odom_msg.twist = twist_odom_msg
    return odom_msg


def get_wifi_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> WiFiState:
    """Maps wireless state data from robot state proto to ROS WiFiState message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        WiFiState message
    """
    wifi_msg = WiFiState()
    for comm_state in state.comms_states:
        if comm_state.HasField("wifi_state"):
            wifi_msg.current_mode = comm_state.wifi_state.current_mode
            wifi_msg.essid = comm_state.wifi_state.essid

    return wifi_msg


def get_tf_from_state(
    state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper, inverse_target_frame: str
) -> TFMessage:
    """Maps robot link state data from robot state proto to ROS TFMessage message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
        inverse_target_frame: A frame name to be inversed to a parent frame.
    Returns:
        TFMessage message
    """
    tf_msg = TFMessage()

    for frame_name in state.kinematic_state.transforms_snapshot.child_to_parent_edge_map:
        if state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
            try:
                transform = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
                local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
                tf_time = Time(sec=local_time.seconds, nanosec=local_time.nanos)
                if inverse_target_frame == spot_wrapper.frame_prefix + frame_name:
                    geo_tform_inversed = SE3Pose.from_obj(transform.parent_tform_child).inverse()
                    new_tf = populate_transform_stamped(
                        tf_time, frame_name, transform.parent_frame_name, geo_tform_inversed, spot_wrapper.frame_prefix
                    )
                else:
                    new_tf = populate_transform_stamped(
                        tf_time,
                        transform.parent_frame_name,
                        frame_name,
                        transform.parent_tform_child,
                        spot_wrapper.frame_prefix,
                    )
                tf_msg.transforms.append(new_tf)
            except Exception as e:
                spot_wrapper.logger.error("Error: {}".format(e))

    return tf_msg


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


def get_battery_states_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> BatteryStateArray:
    """Maps battery state data from robot state proto to ROS BatteryStateArray message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BatteryStateArray message
    """
    battery_states_array_msg = BatteryStateArray()
    for battery in state.battery_states:
        battery_msg = BatteryState()
        local_time = spot_wrapper.robotToLocalTime(battery.timestamp)
        battery_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)

        battery_msg.identifier = battery.identifier
        battery_msg.charge_percentage = battery.charge_percentage.value
        battery_msg.estimated_runtime = Duration(
            sec=battery.estimated_runtime.seconds, nanosec=battery.estimated_runtime.nanos
        )
        battery_msg.current = battery.current.value
        battery_msg.voltage = battery.voltage.value
        for temp in battery.temperatures:
            battery_msg.temperatures.append(temp)
        battery_msg.status = battery.status
        battery_states_array_msg.battery_states.append(battery_msg)

    return battery_states_array_msg


def get_power_states_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> PowerState:
    """Maps power state data from robot state proto to ROS PowerState message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        PowerState message
    """
    power_state_msg = PowerState()
    local_time = spot_wrapper.robotToLocalTime(state.power_state.timestamp)
    power_state_msg.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    power_state_msg.motor_power_state = state.power_state.motor_power_state
    power_state_msg.shore_power_state = state.power_state.shore_power_state
    power_state_msg.locomotion_charge_percentage = state.power_state.locomotion_charge_percentage.value
    power_state_msg.locomotion_estimated_runtime = Duration(
        sec=state.power_state.locomotion_estimated_runtime.seconds,
        nanosec=state.power_state.locomotion_estimated_runtime.nanos,
    )
    return power_state_msg


def get_behavior_faults(behavior_faults: List[BehaviorFault], spot_wrapper: SpotWrapper) -> List[BehaviorFault]:
    """Helper function to strip out behavior faults into a list
    Args:
        behavior_faults: List of BehaviorFaults
        spot_wrapper: A SpotWrapper object
    Returns:
        List of BehaviorFault messages
    """
    faults = []

    for fault in behavior_faults:
        new_fault = BehaviorFault()
        new_fault.behavior_fault_id = fault.behavior_fault_id
        local_time = spot_wrapper.robotToLocalTime(fault.onset_timestamp)
        new_fault.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
        new_fault.cause = fault.cause
        new_fault.status = fault.status
        faults.append(new_fault)

    return faults


def get_system_faults(system_faults: List[SystemFault], spot_wrapper: SpotWrapper) -> List[SystemFault]:
    """Helper function to strip out system faults into a list
    Args:
        system_faults: List of SystemFaults
        spot_wrapper: A SpotWrapper object
    Returns:
        List of SystemFault messages
    """
    faults = []

    for fault in system_faults:
        new_fault = SystemFault()
        new_fault.name = fault.name
        local_time = spot_wrapper.robotToLocalTime(fault.onset_timestamp)
        new_fault.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
        new_fault.duration = Duration(sec=fault.duration.seconds, nanosec=fault.duration.nanos)
        new_fault.code = fault.code
        new_fault.uid = fault.uid
        new_fault.error_message = fault.error_message

        for att in fault.attributes:
            new_fault.attributes.append(att)

        new_fault.severity = fault.severity
        faults.append(new_fault)

    return faults


def get_system_faults_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> SystemFaultState:
    """Maps system fault data from robot state proto to ROS SystemFaultState message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        SystemFaultState message
    """
    system_fault_state_msg = SystemFaultState()
    system_fault_state_msg.faults = get_system_faults(state.system_fault_state.faults, spot_wrapper)
    system_fault_state_msg.historical_faults = get_system_faults(
        state.system_fault_state.historical_faults, spot_wrapper
    )
    return system_fault_state_msg


def get_end_effector_force_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> Vector3Stamped:
    force = Vector3Stamped()
    local_time = spot_wrapper.robotToLocalTime(state.kinematic_state.acquisition_timestamp)
    force.header.stamp = Time(sec=local_time.seconds, nanosec=local_time.nanos)
    force.header.frame_id = spot_wrapper.frame_prefix + "hand"
    force.vector.x = state.manipulator_state.estimated_end_effector_force_in_hand.x
    force.vector.y = state.manipulator_state.estimated_end_effector_force_in_hand.y
    force.vector.z = state.manipulator_state.estimated_end_effector_force_in_hand.z
    return force


def get_behavior_faults_from_state(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> BehaviorFaultState:
    """Maps behavior fault data from robot state proto to ROS BehaviorFaultState message
    Args:
        state: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BehaviorFaultState message
    """
    behavior_fault_state_msg = BehaviorFaultState()
    behavior_fault_state_msg.faults = get_behavior_faults(state.behavior_fault_state.faults, spot_wrapper)
    return behavior_fault_state_msg


def get_from_env_and_fall_back_to_param(
    env_name: str, node: Node, param_name: str, default_value: str
) -> Optional[str]:
    val = os.environ.get(env_name)
    if val is None:
        node.declare_parameter(param_name, default_value)
        val = node.get_parameter(param_name).value
    return val


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
