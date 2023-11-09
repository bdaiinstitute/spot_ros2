// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot/robot.h>

#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {

class RobotApi {
  virtual tl::expected<std::unique_ptr<bosdyn::client::Robot>, std::string> createRobot(
      const std::string& ip_address, const std::string& robot_name) = 0;
};
}  // namespace spot_ros2