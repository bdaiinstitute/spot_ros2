// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot/robot.h>

#include <memory>
#include <string>

namespace spot_ros2 {
class Robot {
 public:
  Robot(std::unique_ptr<::bosdyn::client::Robot> robot, const std::string& robot_name);

  std::string name() const;
  ::bosdyn::client::Robot& robot() const;

 private:
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::string robot_name_;
};
}  // namespace spot_ros2