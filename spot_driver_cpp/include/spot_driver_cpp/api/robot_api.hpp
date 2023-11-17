// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/robot.hpp>

#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {

class RobotApi {
  virtual tl::expected<std::unique_ptr<Robot>, std::string> createRobot(const std::string& ip_address,
                                                                        const std::string& robot_name) const = 0;
};
}  // namespace spot_ros2
