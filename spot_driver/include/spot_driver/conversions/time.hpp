// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/time.hpp>

namespace spot_ros2 {
/**
 * @brief Convert a timestamp that was reported relative to Spot's onboard clock into a timestamp relative to the host's
 * clock.
 *
 * @param timestamp_robot Timestamp relative to Spot's clock.
 * @param clock_skew The difference as measured by Spot between the host's clock and Spot's onboard clock.
 * @return builtin_interfaces::msg::Time Timestamp relative to the host's clock. If applying the clock skew would result
 * in a timestamp earlier than the epoch, return an all-zero timestamp.
 */
builtin_interfaces::msg::Time robotTimeToLocalTime(const google::protobuf::Timestamp& timestamp_robot,
                                                   const google::protobuf::Duration& clock_skew);

/**
 * @brief Convert a timestamp that was reported relative to the host's clock into a timestamp relative to Spot's onboard
 * clock.
 *
 * @param timestamp_local Timestamp relative to the host's clock
 * @param clock_skew The difference as measured by Spot between the host's clock and Spot's onboard clock.
 * @return google::protobuf::Timestamp Timestamp relative to Spot's onboard clock. If applying the clock skew would
 * result in a timestamp earlier than the epoch, return an all-zero timestamp.
 */
google::protobuf::Timestamp localTimeToRobotTime(const builtin_interfaces::msg::Time& timestamp_local,
                                                 const google::protobuf::Duration& clock_skew);
}  // namespace spot_ros2
