// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock-matchers.h>
#include <gmock/gmock-more-matchers.h>
#include <gmock/gmock.h>
#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <bosdyn_msgs/msg/arm_joint_position.hpp>
#include <bosdyn_msgs/msg/inverse_kinematics_request.hpp>
#include <spot_driver/conversions/common_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace {
using ArmJointPosition = bosdyn_msgs::msg::ArmJointPosition;
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
  ASSERT_EQ(rosMsg.client_name = "client_name", protoMsg.client_name());
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

INSTANTIATE_TEST_CASE_P(ConvertBosdynMsgsArmJointPositionToProto, ConvertBosdynMsgsArmJointPositionToProtoParameterized,
                        ::testing::Values(bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.1)
                                              .sh0_is_set(true)
                                              .sh1(0.2)
                                              .sh1_is_set(true)
                                              .el0(0.3)
                                              .el0_is_set(true)
                                              .el1(0.4)
                                              .el1_is_set(true)
                                              .wr0(0.5)
                                              .wr0_is_set(true)
                                              .wr1(0.6)
                                              .wr1_is_set(true),
                                          bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.0)
                                              .sh0_is_set(false)
                                              .sh1(0.2)
                                              .sh1_is_set(true)
                                              .el0(0.3)
                                              .el0_is_set(true)
                                              .el1(0.4)
                                              .el1_is_set(true)
                                              .wr0(0.5)
                                              .wr0_is_set(true)
                                              .wr1(0.6)
                                              .wr1_is_set(true),
                                          bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.1)
                                              .sh0_is_set(true)
                                              .sh1(0.0)
                                              .sh1_is_set(false)
                                              .el0(0.3)
                                              .el0_is_set(true)
                                              .el1(0.4)
                                              .el1_is_set(true)
                                              .wr0(0.5)
                                              .wr0_is_set(true)
                                              .wr1(0.6)
                                              .wr1_is_set(true),
                                          bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.1)
                                              .sh0_is_set(true)
                                              .sh1(0.2)
                                              .sh1_is_set(true)
                                              .el0(0.0)
                                              .el0_is_set(false)
                                              .el1(0.4)
                                              .el1_is_set(true)
                                              .wr0(0.5)
                                              .wr0_is_set(true)
                                              .wr1(0.6)
                                              .wr1_is_set(true),
                                          bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.1)
                                              .sh0_is_set(true)
                                              .sh1(0.2)
                                              .sh1_is_set(true)
                                              .el0(0.3)
                                              .el0_is_set(true)
                                              .el1(0.0)
                                              .el1_is_set(false)
                                              .wr0(0.5)
                                              .wr0_is_set(true)
                                              .wr1(0.6)
                                              .wr1_is_set(true),
                                          bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.1)
                                              .sh0_is_set(true)
                                              .sh1(0.2)
                                              .sh1_is_set(true)
                                              .el0(0.3)
                                              .el0_is_set(true)
                                              .el1(0.4)
                                              .el1_is_set(true)
                                              .wr0(0.0)
                                              .wr0_is_set(false)
                                              .wr1(0.6)
                                              .wr1_is_set(true),
                                          bosdyn_msgs::build<ArmJointPosition>()
                                              .sh0(0.1)
                                              .sh0_is_set(true)
                                              .sh1(0.2)
                                              .sh1_is_set(true)
                                              .el0(0.3)
                                              .el0_is_set(true)
                                              .el1(0.4)
                                              .el1_is_set(true)
                                              .wr0(0.5)
                                              .wr0_is_set(true)
                                              .wr1(0.0)
                                              .wr1_is_set(false)));

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

}  // namespace spot_ros2::test
