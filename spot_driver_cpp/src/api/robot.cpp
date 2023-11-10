// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/robot.hpp>

namespace spot_ros2 {
Robot::Robot(std::unique_ptr<::bosdyn::client::Robot> robot, const std::string& robot_name)
    : robot_{std::move(robot)}, robot_name_{robot_name} {}

std::string Robot::name() const {
  return robot_name_;
}

::bosdyn::client::Robot& Robot::robot() const {
  return *robot_;
}

}  // namespace spot_ros2
