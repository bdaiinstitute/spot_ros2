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
  // StateClientInterface is move-only
  StateClientInterface() = default;
  StateClientInterface(StateClientInterface&& other) = default;
  StateClientInterface(const StateClientInterface&) = delete;
  StateClientInterface& operator=(StateClientInterface&& other) = default;
  StateClientInterface& operator=(const StateClientInterface&) = delete;

  virtual ~StateClientInterface() = default;

  /**
   * @brief Retrieve Spot's most recent robot state data.
   * @return Returns an expected which contains a RobotState message if the request was completed successfully. If the
   * request could not be completed, or if the response does not contain a RobotState message, return an error message
   * describing the failure.
   */
  virtual tl::expected<bosdyn::api::RobotState, std::string> getRobotState() = 0;
};
}  // namespace spot_ros2
