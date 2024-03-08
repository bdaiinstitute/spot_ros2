// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <rclcpp/time.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that calls a callback function at a specified rate.
 */
class ClockInterfaceBase {
 public:
  virtual ~ClockInterfaceBase() = default;

  virtual rclcpp::Time now() = 0;
};
}  // namespace spot_ros2
