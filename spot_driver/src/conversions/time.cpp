// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/conversions/time.hpp>

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/time.hpp>

namespace {
/**
 * @brief Modify seconds and nanoseconds in-place to enforce that the nanoseconds must contain a value between 0 and
 * 999,999,999 by adding or removing whole seconds.
 */
void normalize(int64_t& seconds, int32_t& nanos) {
  if (nanos < 0) {
    nanos += 1e9;
    seconds -= 1;
  } else if (nanos >= 1e9) {
    nanos -= 1e9;
    seconds += 1;
  }
}
}  // namespace

namespace spot_ros2 {
builtin_interfaces::msg::Time robotTimeToLocalTime(const google::protobuf::Timestamp& timestamp_robot,
                                                   const google::protobuf::Duration& clock_skew) {
  int64_t seconds_local = timestamp_robot.seconds() - clock_skew.seconds();
  int32_t nanos_local = timestamp_robot.nanos() - clock_skew.nanos();

  // Carry over a second if needed
  // Note: Since ROS Time messages store the nanoseconds component as an unsigned integer, we need to do this before
  // converting to ROS Time.
  normalize(seconds_local, nanos_local);

  // If the timestamp contains a negative time, create an all-zero ROS Time.
  if (seconds_local < 0) {
    return builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
  }
  return builtin_interfaces::build<builtin_interfaces::msg::Time>()
      .sec(static_cast<int>(seconds_local))
      .nanosec(nanos_local);
}

google::protobuf::Timestamp localTimeToRobotTime(const builtin_interfaces::msg::Time& timestamp_local,
                                                 const google::protobuf::Duration& clock_skew) {
  int64_t seconds_robot = static_cast<int64_t>(timestamp_local.sec) + clock_skew.seconds();
  int32_t nanos_robot = static_cast<int32_t>(timestamp_local.nanosec) + clock_skew.nanos();

  // Carry over a second if needed
  normalize(seconds_robot, nanos_robot);

  // If the timestamp contains a negative time, return an all-zero timestamp
  if (seconds_robot < 0) {
    return google::protobuf::Timestamp{};
  }

  google::protobuf::Timestamp timestamp_robot;
  timestamp_robot.set_seconds(seconds_robot);
  timestamp_robot.set_nanos(nanos_robot);
  return timestamp_robot;
}
}  // namespace spot_ros2
