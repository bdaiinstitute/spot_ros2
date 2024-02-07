// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>

#include <optional>
#include <string>

namespace spot_ros2 {

/**
 * @brief Interface class to interact with Spot SDK's Robot State Client
 */
class RobotStateClientInterface {
 public:
  virtual tl::expected<RobotState, std::string> getRobotState(const std::string& preferred_odom_frame) = 0;
};
}  // namespace spot_ros2
