// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/time.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {

class TimeSyncApi {
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
