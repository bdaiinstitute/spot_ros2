// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2 {

TEST(TestKinematicConversions, convert_bosdyn_msgs_inverse_kinematics_request_to_proto) {
  bosdyn_msgs::msg::InverseKinematicsRequest ros;
  ros.header.client_name = "test_client";
  ros.header_is_set = true;
  ros.header.request_timestamp.sec = 2;
  ros.header.request_timestamp.nanosec = 200;
  ros.header.request_timestamp_is_set = true;
  bosdyn::api::spot::InverseKinematicsRequest proto;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_to_proto(ros, proto);

  ASSERT_EQ(ros.header_is_set, proto.has_header());
  ASSERT_EQ(ros.header.client_name, proto.header().client_name());
  ASSERT_EQ(ros.header.request_timestamp_is_set, proto.header().has_request_timestamp());
  ASSERT_EQ(ros.header.request_timestamp.sec, proto.header().request_timestamp().seconds());
  ASSERT_EQ(ros.header.request_timestamp.nanosec, proto.header().request_timestamp().nanos());
}

}  // namespace spot_ros2