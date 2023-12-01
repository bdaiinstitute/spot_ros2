// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver_cpp/conversions/common_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2::test {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

TEST(TestCommonConversions, convert_builtin_interfaces_time_to_proto) {
  builtin_interfaces::msg::Time ros_msg;
  google::protobuf::Timestamp proto_msg;

  ros_msg.sec = 2;
  ros_msg.nanosec = 200;

  common_conversions::convert_builtin_interfaces_time_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.sec, proto_msg.seconds());
  ASSERT_EQ(ros_msg.nanosec, proto_msg.nanos());
}

TEST(TestCommonConversions, convert_geometry_msgs_vector3_to_proto) {
  geometry_msgs::msg::Vector3 ros_msg;
  bosdyn::api::Vec3 proto_msg;

  ros_msg.x = 0.1;
  ros_msg.y = 0.2;
  ros_msg.z = 0.3;

  common_conversions::convert_geometry_msgs_vector3_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.x, proto_msg.x());
  ASSERT_EQ(ros_msg.y, proto_msg.y());
  ASSERT_EQ(ros_msg.z, proto_msg.z());
}

TEST(TestCommonConversions, convert_geometry_msgs_point_to_proto) {
  geometry_msgs::msg::Point ros_msg;
  bosdyn::api::Vec3 proto_msg;

  ros_msg.x = 0.1;
  ros_msg.y = 0.2;
  ros_msg.z = 0.3;

  common_conversions::convert_geometry_msgs_point_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.x, proto_msg.x());
  ASSERT_EQ(ros_msg.y, proto_msg.y());
  ASSERT_EQ(ros_msg.z, proto_msg.z());
}

TEST(TestCommonConversions, convert_geometry_msgs_quaternion_to_proto) {
  geometry_msgs::msg::Quaternion ros_msg;
  bosdyn::api::Quaternion proto_msg;

  ros_msg.w = 0.7987;
  ros_msg.x = 0.1912;
  ros_msg.y = 0.4336;
  ros_msg.z = 0.3709;

  common_conversions::convert_geometry_msgs_quaternion_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.w, proto_msg.w());
  ASSERT_EQ(ros_msg.x, proto_msg.x());
  ASSERT_EQ(ros_msg.y, proto_msg.y());
  ASSERT_EQ(ros_msg.z, proto_msg.z());
}

TEST(TestCommonConversions, convert_bosdyn_msgs_request_header_to_proto) {
  bosdyn_msgs::msg::RequestHeader ros_msg;
  bosdyn::api::RequestHeader proto_msg;

  ros_msg.request_timestamp_is_set = true;
  ros_msg.client_name = "client_name";
  ros_msg.disable_rpc_logging = true;

  common_conversions::convert_bosdyn_msgs_request_header_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.request_timestamp_is_set, proto_msg.has_request_timestamp());
  ASSERT_EQ(ros_msg.client_name = "client_name", proto_msg.client_name());
  ASSERT_EQ(ros_msg.disable_rpc_logging = true, proto_msg.disable_rpc_logging());
}

TEST(TestCommonConversions, convert_bosdyn_msgs_arm_joint_position_to_proto) {
  bosdyn_msgs::msg::ArmJointPosition ros_msg;
  bosdyn::api::ArmJointPosition proto_msg;

  ros_msg.sh0_is_set = true;
  ros_msg.sh0 = 0.1;
  ros_msg.sh1_is_set = true;
  ros_msg.sh1 = 0.2;
  ros_msg.el0_is_set = true;
  ros_msg.el0 = 0.3;
  ros_msg.el1_is_set = true;
  ros_msg.el1 = 0.4;
  ros_msg.wr0_is_set = true;
  ros_msg.wr0 = 0.5;
  ros_msg.wr1_is_set = true;
  ros_msg.wr1 = 0.6;

  common_conversions::convert_bosdyn_msgs_arm_joint_position_to_proto(ros_msg, proto_msg);

  ASSERT_EQ(ros_msg.sh0_is_set, proto_msg.has_sh0());
  ASSERT_EQ(ros_msg.sh0, proto_msg.sh0().value());
  ASSERT_EQ(ros_msg.sh1_is_set, proto_msg.has_sh1());
  ASSERT_EQ(ros_msg.sh1, proto_msg.sh1().value());
  ASSERT_EQ(ros_msg.el0_is_set, proto_msg.has_el0());
  ASSERT_EQ(ros_msg.el0, proto_msg.el0().value());
  ASSERT_EQ(ros_msg.el1_is_set, proto_msg.has_el1());
  ASSERT_EQ(ros_msg.el1, proto_msg.el1().value());
  ASSERT_EQ(ros_msg.wr0_is_set, proto_msg.has_wr0());
  ASSERT_EQ(ros_msg.wr0, proto_msg.wr0().value());
  ASSERT_EQ(ros_msg.wr1_is_set, proto_msg.has_wr1());
  ASSERT_EQ(ros_msg.wr1, proto_msg.wr1().value());
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

TEST(TestCommonConversions, convert_proto_to_geometry_msgs_quaternion) {
  bosdyn::api::Quaternion proto_msg;
  geometry_msgs::msg::Quaternion ros_msg;

  proto_msg.set_w(0.7987);
  proto_msg.set_x(0.1912);
  proto_msg.set_y(0.4336);
  proto_msg.set_z(0.3709);

  common_conversions::convert_proto_to_geometry_msgs_quaternion(proto_msg, ros_msg);

  ASSERT_EQ(proto_msg.w(), ros_msg.w);
  ASSERT_EQ(proto_msg.x(), ros_msg.x);
  ASSERT_EQ(proto_msg.y(), ros_msg.y);
  ASSERT_EQ(proto_msg.z(), ros_msg.z);
}

}  // namespace spot_ros2::test
