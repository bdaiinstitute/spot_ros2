// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot/robot.h>

#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {
class Robot {
 public:
  Robot(std::unique_ptr<::bosdyn::client::Robot> robot, const std::string& robot_name);

  std::string name() const;

  tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password);
  tl::expected<bool, std::string> hasArm() const;

  ::bosdyn::client::Robot& robot() const;

 private:
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::string robot_name_;
};
}  // namespace spot_ros2