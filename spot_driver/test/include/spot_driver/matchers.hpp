// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock-generated-matchers.h>
#include <gmock/gmock-matchers.h>
#include <gmock/gmock.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <spot_driver/api/time_sync_api.hpp>

/**
 * @brief This verifies that the difference between the input timestamp and the original timestamp matches the output of
 * the applyClockSkew function.
 * @details Don't use this to test applyClockSkew itself, since that would be rather tautological.
 */
MATCHER_P2(ClockSkewIsAppliedToHeader, original_stamp, clock_skew, "") {
  return arg.stamp == spot_ros2::applyClockSkew(original_stamp, clock_skew);
}

MATCHER_P3(GeometryMsgsVector3Eq, x, y, z, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("x", &geometry_msgs::msg::Vector3::x, testing::DoubleEq(x)),
                     testing::Field("y", &geometry_msgs::msg::Vector3::y, testing::DoubleEq(y)),
                     testing::Field("z", &geometry_msgs::msg::Vector3::z, testing::DoubleEq(z))),
      arg, result_listener);
}

MATCHER_P3(GeometryMsgsPointEq, x, y, z, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("x", &geometry_msgs::msg::Point::x, testing::DoubleEq(x)),
                     testing::Field("y", &geometry_msgs::msg::Point::y, testing::DoubleEq(y)),
                     testing::Field("z", &geometry_msgs::msg::Point::z, testing::DoubleEq(z))),
      arg, result_listener);
}

MATCHER_P4(GeometryMsgsQuaternionEq, qw, qx, qy, qz, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("x", &geometry_msgs::msg::Quaternion::x, testing::DoubleEq(qx)),
                     testing::Field("y", &geometry_msgs::msg::Quaternion::y, testing::DoubleEq(qy)),
                     testing::Field("z", &geometry_msgs::msg::Quaternion::z, testing::DoubleEq(qz)),
                     testing::Field("w", &geometry_msgs::msg::Quaternion::w, testing::DoubleEq(qw))),
      arg, result_listener);
}

MATCHER_P7(GeometryMsgsTransformEq, x, y, z, qw, qx, qy, qz, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(
          testing::Field("translation", &geometry_msgs::msg::Transform::translation, GeometryMsgsVector3Eq(x, y, z)),
          testing::Field("rotation", &geometry_msgs::msg::Transform::rotation,
                         GeometryMsgsQuaternionEq(qw, qx, qy, qz))),
      arg, result_listener);
}

MATCHER_P6(GeometryMsgsTwistEq, x, y, z, rx, ry, rz, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("linear", &geometry_msgs::msg::Twist::linear, GeometryMsgsVector3Eq(x, y, z)),
                     testing::Field("angular", &geometry_msgs::msg::Twist::angular, GeometryMsgsVector3Eq(rx, ry, rz))),
      arg, result_listener);
}
