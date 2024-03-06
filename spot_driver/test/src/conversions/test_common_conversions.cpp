// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock-matchers.h>
#include <gmock/gmock-more-matchers.h>
#include <gmock/gmock.h>
#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <bosdyn_api_msgs/msg/arm_joint_position.hpp>
#include <spot_driver/conversions/common_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace {
using ArmJointPosition = bosdyn_api_msgs::msg::ArmJointPosition;
}

namespace spot_ros2::test {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

TEST(TestCommonConversions, convert_builtin_interfaces_time_to_proto) {
  builtin_interfaces::msg::Time rosMsg;
  google::protobuf::Timestamp protoMsg;

  rosMsg.sec = 2;
  rosMsg.nanosec = 200;

  convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.sec, protoMsg.seconds());
  ASSERT_EQ(static_cast<int>(rosMsg.nanosec), protoMsg.nanos());
}

TEST(TestCommonConversions, convertGeometryMsgsVector3ToProto) {
  geometry_msgs::msg::Vector3 rosMsg;
  bosdyn::api::Vec3 protoMsg;

  rosMsg.x = 0.1;
  rosMsg.y = 0.2;
  rosMsg.z = 0.3;

  convertToProto(rosMsg, protoMsg);

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

  convertToProto(rosMsg, protoMsg);

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

  convertToProto(rosMsg, protoMsg);

  ASSERT_EQ(rosMsg.w, protoMsg.w());
  ASSERT_EQ(rosMsg.x, protoMsg.x());
  ASSERT_EQ(rosMsg.y, protoMsg.y());
  ASSERT_EQ(rosMsg.z, protoMsg.z());
}

TEST(TestCommonConversions, convertBosdynMsgsRequestHeaderToProto) {
  bosdyn_api_msgs::msg::RequestHeader rosMsg;
  bosdyn::api::RequestHeader protoMsg;

  // All fields set by default.
  rosMsg.client_name = "client_name";
  rosMsg.disable_rpc_logging = true;

  convertToProto(rosMsg, protoMsg);

  ASSERT_TRUE(protoMsg.has_request_timestamp());
  ASSERT_EQ(rosMsg.client_name, protoMsg.client_name());
  ASSERT_EQ(rosMsg.disable_rpc_logging, protoMsg.disable_rpc_logging());
}

class ConvertBosdynMsgsArmJointPositionToProtoParameterized : public ::testing::TestWithParam<ArmJointPosition> {};

TEST_P(ConvertBosdynMsgsArmJointPositionToProtoParameterized, CheckFieldsNotSet) {
  // GIVEN an arm joint position message with some number of fields set or not set
  ArmJointPosition ros_msg = GetParam();
  bosdyn::api::ArmJointPosition proto_msg;
  // WHEN converting to a protobuf message
  convertToProto(ros_msg, proto_msg);

  // THEN we expect that each corresponding field is set and has the same value
  const bool sh0_is_set = (ros_msg.has_field & ArmJointPosition::SH0_FIELD_SET) != 0;
  EXPECT_THAT(proto_msg.has_sh0(), testing::Eq(sh0_is_set));
  if (sh0_is_set) {
    EXPECT_THAT(proto_msg.sh0().value(), testing::DoubleEq(ros_msg.sh0.data));
  }

  const bool sh1_is_set = (ros_msg.has_field & ArmJointPosition::SH1_FIELD_SET) != 0;
  EXPECT_THAT(proto_msg.has_sh1(), testing::Eq(sh1_is_set));
  if (sh1_is_set) {
    EXPECT_THAT(proto_msg.sh1().value(), testing::DoubleEq(ros_msg.sh1.data));
  }

  const bool el0_is_set = (ros_msg.has_field & ArmJointPosition::EL0_FIELD_SET) != 0;
  EXPECT_THAT(proto_msg.has_el0(), testing::Eq(el0_is_set));
  if (el0_is_set) {
    EXPECT_THAT(proto_msg.el0().value(), testing::DoubleEq(ros_msg.el0.data));
  }

  const bool el1_is_set = (ros_msg.has_field & ArmJointPosition::EL1_FIELD_SET) != 0;
  EXPECT_THAT(proto_msg.has_el1(), testing::Eq(el1_is_set));
  if (el1_is_set) {
    EXPECT_THAT(proto_msg.el1().value(), testing::DoubleEq(ros_msg.el1.data));
  }

  const bool wr0_is_set = (ros_msg.has_field & ArmJointPosition::WR0_FIELD_SET) != 0;
  EXPECT_THAT(proto_msg.has_wr0(), testing::Eq(wr0_is_set));
  if (wr0_is_set) {
    EXPECT_THAT(proto_msg.wr0().value(), testing::DoubleEq(ros_msg.wr0.data));
  }

  const bool wr1_is_set = (ros_msg.has_field & ArmJointPosition::WR1_FIELD_SET) != 0;
  EXPECT_THAT(proto_msg.has_wr1(), testing::Eq(wr1_is_set));
  if (wr1_is_set) {
    EXPECT_THAT(proto_msg.wr1().value(), testing::DoubleEq(ros_msg.wr1.data));
  }
}

std::vector<ArmJointPosition> BuildArmJointPositionsForTesting() {
  ArmJointPosition default_arm_joint_position;
  default_arm_joint_position.sh0.data = 0.1;
  default_arm_joint_position.sh1.data = 0.2;
  default_arm_joint_position.el0.data = 0.3;
  default_arm_joint_position.el1.data = 0.4;
  default_arm_joint_position.wr0.data = 0.5;
  default_arm_joint_position.wr1.data = 0.6;

  std::vector<ArmJointPosition> values(7, default_arm_joint_position);
  values[1].has_field &= ~ArmJointPosition::SH0_FIELD_SET;
  values[2].has_field &= ~ArmJointPosition::SH1_FIELD_SET;
  values[3].has_field &= ~ArmJointPosition::EL0_FIELD_SET;
  values[4].has_field &= ~ArmJointPosition::EL1_FIELD_SET;
  values[5].has_field &= ~ArmJointPosition::WR0_FIELD_SET;
  values[6].has_field &= ~ArmJointPosition::WR1_FIELD_SET;
  return values;
}

INSTANTIATE_TEST_SUITE_P(ConvertBosdynMsgsArmJointPositionToProto,
                         ConvertBosdynMsgsArmJointPositionToProtoParameterized,
                         ::testing::ValuesIn(BuildArmJointPositionsForTesting()));

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

TEST(TestCommonConversions, convertProtoToBuiltinInterfacesTime) {
  google::protobuf::Timestamp protoMsg;
  builtin_interfaces::msg::Time rosMsg;

  protoMsg.set_seconds(5);
  protoMsg.set_nanos(200);

  convertToRos(protoMsg, rosMsg);

  ASSERT_EQ(protoMsg.seconds(), rosMsg.sec);
  ASSERT_EQ(protoMsg.nanos(), static_cast<int>(rosMsg.nanosec));
}

TEST(TestCommonConversions, convertProtoToGeometryMsgsVector3) {
  bosdyn::api::Vec3 protoMsg;
  geometry_msgs::msg::Point rosMsg;

  protoMsg.set_x(0.1);
  protoMsg.set_y(0.2);
  protoMsg.set_z(0.3);

  convertToRos(protoMsg, rosMsg);

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

  convertToRos(protoMsg, rosMsg);

  ASSERT_EQ(protoMsg.w(), rosMsg.w);
  ASSERT_EQ(protoMsg.x(), rosMsg.x);
  ASSERT_EQ(protoMsg.y(), rosMsg.y);
  ASSERT_EQ(protoMsg.z(), rosMsg.z);
}

TEST(TestCommonConversions, convertProtoToBosdynMsgsRequestHeader) {
  bosdyn::api::RequestHeader protoMsg;
  bosdyn_api_msgs::msg::RequestHeader rosMsg;

  protoMsg.mutable_request_timestamp()->set_seconds(1);
  protoMsg.set_client_name("client_name");
  protoMsg.set_disable_rpc_logging(true);

  convertToRos(protoMsg, rosMsg);

  ASSERT_TRUE(rosMsg.has_field & bosdyn_api_msgs::msg::RequestHeader::REQUEST_TIMESTAMP_FIELD_SET);
  ASSERT_EQ(protoMsg.request_timestamp().seconds(), rosMsg.request_timestamp.sec);
  ASSERT_EQ(protoMsg.client_name(), rosMsg.client_name);
  ASSERT_EQ(protoMsg.disable_rpc_logging(), rosMsg.disable_rpc_logging);
}

TEST(TestCommonConversions, convertProtoToBosdynMsgsResponseHeader) {
  bosdyn::api::ResponseHeader protoMsg;
  bosdyn_api_msgs::msg::ResponseHeader rosMsg;

  protoMsg.mutable_request_header()->set_client_name("client_name");
  protoMsg.mutable_request_received_timestamp()->set_seconds(1);
  protoMsg.mutable_response_timestamp()->set_seconds(2);
  protoMsg.mutable_error()->set_message("ok");

  convertToRos(protoMsg, rosMsg);

  using ResponseHeader = bosdyn_api_msgs::msg::ResponseHeader;
  ASSERT_TRUE(rosMsg.has_field & ResponseHeader::REQUEST_HEADER_FIELD_SET);
  ASSERT_TRUE(rosMsg.has_field & ResponseHeader::REQUEST_RECEIVED_TIMESTAMP_FIELD_SET);
  ASSERT_TRUE(rosMsg.has_field & ResponseHeader::RESPONSE_TIMESTAMP_FIELD_SET);
  ASSERT_TRUE(rosMsg.has_field & ResponseHeader::ERROR_FIELD_SET);

  ASSERT_EQ(protoMsg.request_header().client_name(), rosMsg.request_header.client_name);
  ASSERT_EQ(protoMsg.request_received_timestamp().seconds(), rosMsg.request_received_timestamp.sec);
  ASSERT_EQ(protoMsg.response_timestamp().seconds(), rosMsg.response_timestamp.sec);
  ASSERT_EQ(protoMsg.error().message(), rosMsg.error.message);
}

}  // namespace spot_ros2::test
