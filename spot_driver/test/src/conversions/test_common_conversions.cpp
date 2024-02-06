// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver/conversions/common_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2::test {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

TEST(TestCommonConversions, convert_builtin_interfaces_time_to_proto) {
  builtin_interfaces::msg::Time rosMsg;
  google::protobuf::Timestamp protoMsg;

  rosMsg.sec = 2;
  rosMsg.nanosec = 200;

  common_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.sec, protoMsg.seconds());
  ASSERT_EQ(rosMsg.nanosec, protoMsg.nanos());
}

TEST(TestCommonConversions, convertGeometryMsgsVector3ToProto) {
  geometry_msgs::msg::Vector3 rosMsg;
  bosdyn::api::Vec3 protoMsg;

  rosMsg.x = 0.1;
  rosMsg.y = 0.2;
  rosMsg.z = 0.3;

  common_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.x, protoMsg.x());
  ASSERT_EQ(rosMsg.y, protoMsg.y());
  ASSERT_EQ(rosMsg.z, protoMsg.z());
}

TEST(TestCommonConversions, convertGeometryMsgsPointToProto) {
  geometry_msgs::msg::Point rosMsg;
  bosdyn::api::Vec3 protoMsg;

  rosMsg.x = 0.1;
  rosMsg.y = 0.2;
  rosMsg.z = 0.3;

  common_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.x, protoMsg.x());
  ASSERT_EQ(rosMsg.y, protoMsg.y());
  ASSERT_EQ(rosMsg.z, protoMsg.z());
}

TEST(TestCommonConversions, convertGeometryMsgsQuaternionToProto) {
  geometry_msgs::msg::Quaternion rosMsg;
  bosdyn::api::Quaternion protoMsg;

  rosMsg.w = 0.7987;
  rosMsg.x = 0.1912;
  rosMsg.y = 0.4336;
  rosMsg.z = 0.3709;

  common_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.w, protoMsg.w());
  ASSERT_EQ(rosMsg.x, protoMsg.x());
  ASSERT_EQ(rosMsg.y, protoMsg.y());
  ASSERT_EQ(rosMsg.z, protoMsg.z());
}

TEST(TestCommonConversions, convertBosdynMsgsRequestHeaderToProto) {
  bosdyn_msgs::msg::RequestHeader rosMsg;
  bosdyn::api::RequestHeader protoMsg;

  rosMsg.request_timestamp_is_set = true;
  rosMsg.client_name = "client_name";
  rosMsg.disable_rpc_logging = true;

  common_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.request_timestamp_is_set, protoMsg.has_request_timestamp());
  ASSERT_EQ(rosMsg.client_name = "client_name", protoMsg.client_name());
  ASSERT_EQ(rosMsg.disable_rpc_logging = true, protoMsg.disable_rpc_logging());
}

TEST(TestCommonConversions, convertBosdynMsgsArmJointPositionToProto) {
  bosdyn_msgs::msg::ArmJointPosition rosMsg;
  bosdyn::api::ArmJointPosition protoMsg;

  rosMsg.sh0_is_set = true;
  rosMsg.sh0 = 0.1;
  rosMsg.sh1_is_set = true;
  rosMsg.sh1 = 0.2;
  rosMsg.el0_is_set = true;
  rosMsg.el0 = 0.3;
  rosMsg.el1_is_set = true;
  rosMsg.el1 = 0.4;
  rosMsg.wr0_is_set = true;
  rosMsg.wr0 = 0.5;
  rosMsg.wr1_is_set = true;
  rosMsg.wr1 = 0.6;

  common_conversions::convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.sh0_is_set, protoMsg.has_sh0());
  ASSERT_EQ(rosMsg.sh0, protoMsg.sh0().value());
  ASSERT_EQ(rosMsg.sh1_is_set, protoMsg.has_sh1());
  ASSERT_EQ(rosMsg.sh1, protoMsg.sh1().value());
  ASSERT_EQ(rosMsg.el0_is_set, protoMsg.has_el0());
  ASSERT_EQ(rosMsg.el0, protoMsg.el0().value());
  ASSERT_EQ(rosMsg.el1_is_set, protoMsg.has_el1());
  ASSERT_EQ(rosMsg.el1, protoMsg.el1().value());
  ASSERT_EQ(rosMsg.wr0_is_set, protoMsg.has_wr0());
  ASSERT_EQ(rosMsg.wr0, protoMsg.wr0().value());
  ASSERT_EQ(rosMsg.wr1_is_set, protoMsg.has_wr1());
  ASSERT_EQ(rosMsg.wr1, protoMsg.wr1().value());
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

TEST(TestCommonConversions, convertProtoToBuiltinInterfacesTime) {
  google::protobuf::Timestamp protoMsg;
  builtin_interfaces::msg::Time rosMsg;

  protoMsg.set_seconds(5);
  protoMsg.set_nanos(200);

  common_conversions::convertToRos(protoMsg, rosMsg);

  ASSERT_EQ(protoMsg.seconds(), rosMsg.sec);
  ASSERT_EQ(protoMsg.nanos(), rosMsg.nanosec);
}

TEST(TestCommonConversions, convertProtoToGeometryMsgsVector3) {
  bosdyn::api::Vec3 protoMsg;
  geometry_msgs::msg::Point rosMsg;

  protoMsg.set_x(0.1);
  protoMsg.set_y(0.2);
  protoMsg.set_z(0.3);

  common_conversions::convertToRos(protoMsg, rosMsg);

  ASSERT_EQ(protoMsg.x(), rosMsg.x);
  ASSERT_EQ(protoMsg.y(), rosMsg.y);
  ASSERT_EQ(protoMsg.z(), rosMsg.z);
}

TEST(TestCommonConversions, convertProtoToGeometryMsgsQuaternion) {
  bosdyn::api::Quaternion protoMsg;
  geometry_msgs::msg::Quaternion rosMsg;

  protoMsg.set_w(0.7987);
  protoMsg.set_x(0.1912);
  protoMsg.set_y(0.4336);
  protoMsg.set_z(0.3709);

  common_conversions::convertToRos(protoMsg, rosMsg);

  ASSERT_EQ(protoMsg.w(), rosMsg.w);
  ASSERT_EQ(protoMsg.x(), rosMsg.x);
  ASSERT_EQ(protoMsg.y(), rosMsg.y);
  ASSERT_EQ(protoMsg.z(), rosMsg.z);
}

}  // namespace spot_ros2::test
