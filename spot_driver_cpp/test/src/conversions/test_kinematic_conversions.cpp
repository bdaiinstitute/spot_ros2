// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2 {

TEST(TestKinematicConversions, my_test) {
  bosdyn_msgs::msg::InverseKinematicsRequest ros_msg;
  bosdyn::api::spot::InverseKinematicsRequest proto;

  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_to_proto(ros_msg, proto);
}

}  // namespace spot_ros2