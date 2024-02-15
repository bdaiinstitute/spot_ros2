// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock-generated-matchers.h>
#include <gmock/gmock-matchers.h>
#include <gmock/gmock.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_msgs/msg/e_stop_state.hpp>
#include <spot_msgs/msg/system_fault.hpp>

namespace spot_ros2::test {
/**
 * @brief This verifies that the difference between the input timestamp and the original timestamp matches the output of
 * the applyClockSkew function.
 * @details Don't use this to test applyClockSkew itself, since that would be rather tautological.
 */
MATCHER_P2(ClockSkewIsAppliedToHeader, original_stamp, clock_skew, "") {
  return testing::ExplainMatchResult(testing::Eq(spot_ros2::applyClockSkew(original_stamp, clock_skew)), arg.stamp,
                                     result_listener);
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

MATCHER_P7(GeometryMsgsPoseEq, x, y, z, qw, qx, qy, qz, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("position", &geometry_msgs::msg::Pose::position, GeometryMsgsPointEq(x, y, z)),
                     testing::Field("orientation", &geometry_msgs::msg::Pose::orientation,
                                    GeometryMsgsQuaternionEq(qw, qx, qy, qz))),
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

MATCHER_P6(EStopStateEq, name, type, state, state_description, timestamp, clock_skew, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("name", &spot_msgs::msg::EStopState::name, testing::StrEq(name)),
                     testing::Field("type", &spot_msgs::msg::EStopState::type, testing::Eq(type)),
                     testing::Field("state", &spot_msgs::msg::EStopState::state, testing::Eq(state)),
                     testing::Field("state_description", &spot_msgs::msg::EStopState::state_description,
                                    testing::StrEq(state_description)),
                     testing::Field("header", &spot_msgs::msg::EStopState::header,
                                    ClockSkewIsAppliedToHeader(timestamp, clock_skew))),
      arg, result_listener);
}

MATCHER_P6(EStopStatesContains, name, type, state, state_description, timestamp, clock_skew, "") {
  return testing::ExplainMatchResult(
      testing::Contains(EStopStateEq(name, type, state, state_description, timestamp, clock_skew)), arg,
      result_listener);
}

MATCHER_P2(DurationEq, sec, nanosec, "") {
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("sec", &builtin_interfaces::msg::Duration::sec, testing::Eq(sec)),
                     testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(nanosec))),
      arg, result_listener);
}

MATCHER_P7(SystemFaultIs, header_matcher, duration_matcher, name_matcher, uid_matcher, code_matcher, error_msg_matcher,
           attributes_matcher, "") {
  using SystemFault = spot_msgs::msg::SystemFault;
  return testing::ExplainMatchResult(
      testing::AllOf(testing::Field("header", &SystemFault::header, header_matcher),
                     testing::Field("duration", &SystemFault::duration, duration_matcher),
                     testing::Field("name", &SystemFault::name, name_matcher),
                     testing::Field("code", &SystemFault::code, code_matcher),
                     testing::Field("uid", &SystemFault::uid, uid_matcher),
                     testing::Field("error_message", &SystemFault::error_message, error_msg_matcher),
                     testing::Field("attributes", &SystemFault::attributes, attributes_matcher)),
      arg, result_listener);
}
}  // namespace spot_ros2::test
