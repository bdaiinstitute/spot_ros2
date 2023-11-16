// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/time.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {

inline builtin_interfaces::msg::Time applyClockSkew(const google::protobuf::Timestamp& timestamp,
                                                    const google::protobuf::Duration& clock_skew) {
  int64_t seconds_unskewed = timestamp.seconds() - clock_skew.seconds();
  int32_t nanos_unskewed = timestamp.nanos() - clock_skew.nanos();

  // Carry over a second if needed
  // Note: Since ROS Time messages store the nanoseconds component as an unsigned integer, we need to do this before
  // converting to ROS Time.
  if (nanos_unskewed < 0) {
    nanos_unskewed += 1e9;
    seconds_unskewed -= 1;
  } else if (nanos_unskewed >= 1e9) {
    nanos_unskewed -= 1e9;
    seconds_unskewed += 1;
  }

  // If the timestamp contains a negative time, create an all-zero ROS Time.
  if (seconds_unskewed < 0) {
    return builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
  } else {
    return builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(seconds_unskewed).nanosec(nanos_unskewed);
  }
}

class TimeSyncApi {
 public:
  virtual tl::expected<builtin_interfaces::msg::Time, std::string> convertRobotTimeToLocalTime(
      const google::protobuf::Timestamp& robot_timestamp) = 0;

  /**
  * @brief Get the current clock skew from the Spot SDK's time sync endpoint.
  * @details The clock skew is the difference between Spot's internal system clock and the host PC's system clock.
  * For example, a clock skew of 1.0 seconds means that a timestamp from Spot's clock that is 10.0 seconds after epoch
  * would correspond to a timestmap from the system clock that is 9.0 seconds after epoch.

  * The Spot SDK documentation provides a more detailed explanation of how Spot's time sync works here:
  * https://dev.bostondynamics.com/docs/concepts/base_services#time-sync
  *
  * @return If the clock skew was successfully calculated, return a Duration containing the difference between Spot's
  * internal clock and the host's system clock.
  * @return If the Spot SDK's time sync thread was not initialized, return an error message.
  * @return If the Spot SDK's time sync endpoint fails to handle the clock skew request, return an error message.
  */
  virtual tl::expected<google::protobuf::Duration, std::string> getClockSkew() = 0;
};
}  // namespace spot_ros2
