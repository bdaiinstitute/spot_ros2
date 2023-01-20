from bosdyn.client.math_helpers import SE3Pose, Quat
from geometry_msgs.msg import Point, Pose2D, Pose, Quaternion  # Note: Pose2D is deprecated


# ROS <-> BOSDYN MATH

def ros_transform_to_se3_pose(transform):
    return SE3Pose(transform.translation.x, transform.translation.y, transform.translation.z,
                   Quat(transform.rotation.w, transform.rotation.x, transform.rotation.y,
                        transform.rotation.z))

def ros_pose_to_se3_pose(pose):
    return SE3Pose(pose.position.x, pose.position.y, pose.position.z,
                   Quat(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                        pose.orientation.z))

def se2_pose_to_ros_pose2(se2_pose):
    return Pose2D(x=se2_pose.x, y=se2_pose.y, theta=se2_pose.angle)

def se3_pose_to_ros_pose(se3_pose):
    return Pose(position=Point(x=float(se3_pose.x), y=float(se3_pose.y), z=float(se3_pose.z)),
                orientation=Quaternion(w=float(se3_pose.rot.w), x=float(se3_pose.rot.x), y=float(se3_pose.rot.y),
                                       z=float(se3_pose.rot.z)))

# ROS <-> PROTO

def convert_builtin_interfaces_duration_to_proto(ros_duration, proto_duration):
    proto_duration.seconds = ros_duration.sec
    proto_duration.nanos = ros_duration.nanosec

def convert_proto_to_builtin_interfaces_duration(proto, ros_msg):
    ros_msg.sec = proto.seconds
    ros_msg.nanosec = proto.nanos

def convert_builtin_interfaces_time_to_proto(ros_time, proto_time):
    proto_time.seconds = ros_time.sec
    proto_time.nanos = ros_time.nanosec

def convert_proto_to_builtin_interfaces_time(proto, ros_msg):
    ros_msg.sec = proto.seconds
    ros_msg.nanosec = proto.nanos


def convert_geometry_msgs_vector3_to_proto(ros_point, proto_point):
    proto_point.x = ros_point.x
    proto_point.y = ros_point.y
    proto_point.z = ros_point.z

def convert_proto_to_geometry_msgs_vector3(proto, ros_msg):
    ros_msg.x = proto.x
    ros_msg.y = proto.y
    ros_msg.z = proto.z

def convert_geometry_msgs_quaternion_to_proto(ros_quat, proto_quat):
    proto_quat.w = ros_quat.w
    proto_quat.x = ros_quat.x
    proto_quat.y = ros_quat.y
    proto_quat.z = ros_quat.z

def convert_proto_to_geometry_msgs_quaternion(proto, ros_msg):
    ros_msg.w = proto.w
    ros_msg.x = proto.x
    ros_msg.y = proto.y
    ros_msg.z = proto.z


def convert_geometry_msgs_pose_to_proto(ros_pose, proto_pose):
    convert_geometry_msgs_vector3_to_proto(ros_pose.position, proto_pose.position)
    convert_geometry_msgs_quaternion_to_proto(ros_pose.orientation, proto_pose.rotation)

def convert_proto_to_geometry_msgs_pose(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.position, ros_msg.position)
    convert_proto_to_geometry_msgs_quaternion(proto.rotation, ros_msg.orientation)

def convert_geometry_msgs_twist_to_proto(ros_twist, proto_twist):
    convert_geometry_msgs_vector3_to_proto(ros_twist.linear, proto_twist.linear)
    convert_geometry_msgs_vector3_to_proto(ros_twist.angular, proto_twist.angular)

def convert_proto_to_geometry_msgs_twist(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.linear, ros_msg.linear)
    convert_proto_to_geometry_msgs_vector3(proto.angular, ros_msg.angular)

def convert_geometry_msgs_wrench_to_proto(ros_msg, proto):
    convert_geometry_msgs_vector3_to_proto(ros_msg.force, proto.force)
    convert_geometry_msgs_vector3_to_proto(ros_msg.torque, proto.torque)

def convert_proto_to_geometry_msgs_wrench(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.force, ros_msg.force)
    convert_proto_to_geometry_msgs_vector3(proto.torque, ros_msg.torque)

