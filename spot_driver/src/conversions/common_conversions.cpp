// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/conversions/common_conversions.hpp>

namespace spot_ros2 {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convertToProto(const builtin_interfaces::msg::Time& ros_msg, google::protobuf::Timestamp& proto) {
  proto.set_seconds(ros_msg.sec);
  proto.set_nanos(ros_msg.nanosec);
}

void convertToProto(const bosdyn_api_msgs::msg::RequestHeader& ros_msg, bosdyn::api::RequestHeader& proto) {
  if (ros_msg.has_field & bosdyn_api_msgs::msg::RequestHeader::REQUEST_TIMESTAMP_FIELD_SET) {
    convertToProto(ros_msg.request_timestamp, *proto.mutable_request_timestamp());
  }
  proto.set_client_name(ros_msg.client_name);
  proto.set_disable_rpc_logging(ros_msg.disable_rpc_logging);
}

void convertToProto(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3& proto) {
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convertToProto(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3& proto) {
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convertToProto(const geometry_msgs::msg::Quaternion& ros_msg, bosdyn::api::Quaternion& proto) {
  proto.set_w(ros_msg.w);
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convertToProto(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE3Pose& proto) {
  convertToProto(ros_msg.position, *proto.mutable_position());
  convertToProto(ros_msg.orientation, *proto.mutable_rotation());
}

void convertToProto(const geometry_msgs::msg::Transform& ros_msg, bosdyn::api::SE3Pose& proto) {
  convertToProto(ros_msg.translation, *proto.mutable_position());
  convertToProto(ros_msg.rotation, *proto.mutable_rotation());
}

void convertToProto(const std_msgs::msg::Float64& ros_msg, google::protobuf::DoubleValue& proto) {
  proto.set_value(ros_msg.data);
}

void convertToProto(const bosdyn_api_msgs::msg::ArmJointPosition& ros_msg, bosdyn::api::ArmJointPosition& proto) {
  if (ros_msg.has_field & bosdyn_api_msgs::msg::ArmJointPosition::SH0_FIELD_SET) {
    convertToProto(ros_msg.sh0, *proto.mutable_sh0());
  }
  if (ros_msg.has_field & bosdyn_api_msgs::msg::ArmJointPosition::SH1_FIELD_SET) {
    convertToProto(ros_msg.sh1, *proto.mutable_sh1());
  }
  if (ros_msg.has_field & bosdyn_api_msgs::msg::ArmJointPosition::EL0_FIELD_SET) {
    convertToProto(ros_msg.el0, *proto.mutable_el0());
  }
  if (ros_msg.has_field & bosdyn_api_msgs::msg::ArmJointPosition::EL1_FIELD_SET) {
    convertToProto(ros_msg.el1, *proto.mutable_el1());
  }
  if (ros_msg.has_field & bosdyn_api_msgs::msg::ArmJointPosition::WR0_FIELD_SET) {
    convertToProto(ros_msg.wr0, *proto.mutable_wr0());
  }
  if (ros_msg.has_field & bosdyn_api_msgs::msg::ArmJointPosition::WR1_FIELD_SET) {
    convertToProto(ros_msg.wr1, *proto.mutable_wr1());
  }
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convertToRos(const bosdyn::api::RequestHeader& proto, bosdyn_api_msgs::msg::RequestHeader& ros_msg) {
  ros_msg.has_field = 0u;
  if (proto.has_request_timestamp()) {
    convertToRos(proto.request_timestamp(), ros_msg.request_timestamp);
    ros_msg.has_field |= bosdyn_api_msgs::msg::RequestHeader::REQUEST_TIMESTAMP_FIELD_SET;
  }
  ros_msg.client_name = proto.client_name();
  ros_msg.disable_rpc_logging = proto.disable_rpc_logging();
}

void convertToRos(const bosdyn::api::CommonError& proto, bosdyn_api_msgs::msg::CommonError& ros_msg) {
  ros_msg.code.value = proto.code();
  ros_msg.message = proto.message();
}

void convertToRos(const bosdyn::api::ResponseHeader& proto, bosdyn_api_msgs::msg::ResponseHeader& ros_msg) {
  ros_msg.has_field = 0u;
  if (proto.has_request_header()) {
    convertToRos(proto.request_header(), ros_msg.request_header);
    ros_msg.has_field |= bosdyn_api_msgs::msg::ResponseHeader::REQUEST_HEADER_FIELD_SET;
  }
  if (proto.has_request_received_timestamp()) {
    convertToRos(proto.request_received_timestamp(), ros_msg.request_received_timestamp);
    ros_msg.has_field |= bosdyn_api_msgs::msg::ResponseHeader::REQUEST_RECEIVED_TIMESTAMP_FIELD_SET;
  }
  if (proto.has_response_timestamp()) {
    convertToRos(proto.response_timestamp(), ros_msg.response_timestamp);
    ros_msg.has_field |= bosdyn_api_msgs::msg::ResponseHeader::RESPONSE_TIMESTAMP_FIELD_SET;
  }
  if (proto.has_error()) {
    convertToRos(proto.error(), ros_msg.error);
    ros_msg.has_field |= bosdyn_api_msgs::msg::ResponseHeader::ERROR_FIELD_SET;
  }
}

void convertToRos(const google::protobuf::Timestamp& proto, builtin_interfaces::msg::Time& ros_msg) {
  ros_msg.sec = proto.seconds();
  ros_msg.nanosec = proto.nanos();
}

void convertToRos(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Vector3& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convertToRos(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Point& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convertToRos(const bosdyn::api::Quaternion& proto, geometry_msgs::msg::Quaternion& ros_msg) {
  ros_msg.w = proto.w();
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convertToRos(const bosdyn::api::SE3Pose& proto, geometry_msgs::msg::Pose& ros_msg) {
  convertToRos(proto.position(), ros_msg.position);
  convertToRos(proto.rotation(), ros_msg.orientation);
}

void convertToRos(const bosdyn::api::SE3Pose& proto, geometry_msgs::msg::Transform& ros_msg) {
  convertToRos(proto.position(), ros_msg.translation);
  convertToRos(proto.rotation(), ros_msg.rotation);
}

void convertToRos(const bosdyn::api::SE3Velocity& proto, geometry_msgs::msg::Twist& ros_msg) {
  convertToRos(proto.linear(), ros_msg.linear);
  convertToRos(proto.angular(), ros_msg.angular);
}

}  // namespace spot_ros2
