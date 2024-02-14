// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/robot_state.pb.h>
#include <tl_expected/expected.hpp>

#include <string>

namespace spot_ros2 {

/**
 * @brief Interface class to interact with Spot SDK's Robot State Client
 */
class StateClientInterface {
 public:
  virtual ~StateClientInterface() = default;
  virtual tl::expected<bosdyn::api::RobotState, std::string> getRobotState() = 0;
};
}  // namespace spot_ros2
