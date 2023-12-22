// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/conversions/common_conversions.hpp>

namespace spot_ros2::common_conversions {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convert_builtin_interfaces_time_to_proto(const builtin_interfaces::msg::Time& ros_msg,
                                              google::protobuf::Timestamp& proto) {
  proto.set_seconds(ros_msg.sec);
  proto.set_nanos(ros_msg.nanosec);
}

void convert_bosdyn_msgs_request_header_to_proto(const bosdyn_msgs::msg::RequestHeader& ros_msg,
                                                 bosdyn::api::RequestHeader& proto) {
  if (ros_msg.request_timestamp_is_set) {
    convert_builtin_interfaces_time_to_proto(ros_msg.request_timestamp, *proto.mutable_request_timestamp());
  }
  proto.set_client_name(ros_msg.client_name);
  proto.set_disable_rpc_logging(ros_msg.disable_rpc_logging);
}

void convert_geometry_msgs_vector3_to_proto(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3& proto) {
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convert_geometry_msgs_point_to_proto(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3& proto) {
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convert_geometry_msgs_quaternion_to_proto(const geometry_msgs::msg::Quaternion& ros_msg,
                                               bosdyn::api::Quaternion& proto) {
  proto.set_w(ros_msg.w);
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convert_geometry_msgs_pose_to_proto(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE3Pose& proto) {
  convert_geometry_msgs_point_to_proto(ros_msg.position, *proto.mutable_position());
  convert_geometry_msgs_quaternion_to_proto(ros_msg.orientation, *proto.mutable_rotation());
}

void convert_float64_to_proto(const double ros_msg, google::protobuf::DoubleValue& proto) {
  proto.set_value(ros_msg);
}

void convert_bosdyn_msgs_arm_joint_position_to_proto(const bosdyn_msgs::msg::ArmJointPosition& ros_msg,
                                                     bosdyn::api::ArmJointPosition& proto) {
  if (ros_msg.sh0_is_set) {
    convert_float64_to_proto(ros_msg.sh0, *proto.mutable_sh0());
  }
  if (ros_msg.sh1_is_set) {
    convert_float64_to_proto(ros_msg.sh1, *proto.mutable_sh1());
  }
  if (ros_msg.el0_is_set) {
    convert_float64_to_proto(ros_msg.el0, *proto.mutable_el0());
  }
  if (ros_msg.el1_is_set) {
    convert_float64_to_proto(ros_msg.el1, *proto.mutable_el1());
  }
  if (ros_msg.wr0_is_set) {
    convert_float64_to_proto(ros_msg.wr0, *proto.mutable_wr0());
  }
  if (ros_msg.wr1_is_set) {
    convert_float64_to_proto(ros_msg.wr1, *proto.mutable_wr1());
  }
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convert_proto_to_bosdyn_msgs_request_header(const bosdyn::api::RequestHeader& proto,
                                                 bosdyn_msgs::msg::RequestHeader& ros_msg) {
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.request_timestamp(), ros_msg.request_timestamp);
  ros_msg.request_timestamp_is_set = proto.has_request_timestamp();
  ros_msg.client_name = proto.client_name();
  ros_msg.disable_rpc_logging = proto.disable_rpc_logging();
}

void convert_proto_to_bosdyn_msgs_common_error(const bosdyn::api::CommonError& proto,
                                               bosdyn_msgs::msg::CommonError& ros_msg) {
  ros_msg.code.value = proto.code();
  ros_msg.message = proto.message();
}

void convert_proto_to_bosdyn_msgs_response_header(const bosdyn::api::ResponseHeader& proto,
                                                  bosdyn_msgs::msg::ResponseHeader& ros_msg) {
  convert_proto_to_bosdyn_msgs_request_header(proto.request_header(), ros_msg.request_header);
  ros_msg.request_header_is_set = proto.has_request_header();
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.request_received_timestamp(),
                                                               ros_msg.request_received_timestamp);
  ros_msg.request_received_timestamp_is_set = proto.has_request_received_timestamp();
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.response_timestamp(), ros_msg.response_timestamp);
  ros_msg.response_timestamp_is_set = proto.has_response_timestamp();
  convert_proto_to_bosdyn_msgs_common_error(proto.error(), ros_msg.error);
  ros_msg.error_is_set = proto.has_error();
}

void convert_proto_to_builtin_interfaces_time(const google::protobuf::Timestamp& proto,
                                              builtin_interfaces::msg::Time& ros_msg) {
  ros_msg.sec = proto.seconds();
  ros_msg.nanosec = proto.nanos();
}

void convert_proto_to_geometry_msgs_vector3(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Vector3& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convert_proto_to_geometry_msgs_vector3(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Point& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convert_proto_to_geometry_msgs_quaternion(const bosdyn::api::Quaternion& proto,
                                               geometry_msgs::msg::Quaternion& ros_msg) {
  ros_msg.w = proto.w();
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convert_proto_to_geometry_msgs_pose(const bosdyn::api::SE3Pose& proto, geometry_msgs::msg::Pose& ros_msg) {
  convert_proto_to_geometry_msgs_vector3(proto.position(), ros_msg.position);
  convert_proto_to_geometry_msgs_quaternion(proto.rotation(), ros_msg.orientation);
}

void convert_proto_to_geometry_msgs_twist(const bosdyn::api::SE3Velocity& proto, geometry_msgs::msg::Twist& ros_msg) {
  convert_proto_to_geometry_msgs_vector3(proto.linear(), ros_msg.linear);
  convert_proto_to_geometry_msgs_vector3(proto.angular(), ros_msg.angular);
}

}  // namespace spot_ros2::common_conversions
