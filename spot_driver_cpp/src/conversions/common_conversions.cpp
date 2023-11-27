// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/conversions/common_conversions.hpp>

namespace spot_ros2::common_conversions {

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
  proto.set_disable_rpc_logging(true);
}

void convert_geometry_msgs_vector3_to_proto(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3& proto) {
  proto.set_x(ros_msg.x);
  proto.set_y(ros_msg.y);
  proto.set_z(ros_msg.z);
}

void convert_geometry_msgs_vector3_to_proto(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3& proto) {
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
  convert_geometry_msgs_vector3_to_proto(ros_msg.position, *proto.mutable_position());
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
}  // namespace spot_ros2::common_conversions