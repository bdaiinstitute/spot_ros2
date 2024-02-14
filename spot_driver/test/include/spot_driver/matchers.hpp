// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>
#include <spot_driver/api/time_sync_api.hpp>

/**
 * @brief This verifies that the difference between the input timestamp and the original timestamp matches the output of
 * the applyClockSkew function.
 * @details Don't use this to test applyClockSkew itself, since that would be rather tautological.
 */
MATCHER_P2(ClockSkewIsAppliedToHeader, original_stamp, clock_skew, "") {
  return arg.stamp == spot_ros2::applyClockSkew(original_stamp, clock_skew);
}
