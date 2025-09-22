// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that gets the current timestamp from a clock.
 */
class ClockInterfaceBase {
 public:
  virtual ~ClockInterfaceBase() = default;

  virtual std::shared_ptr<rclcpp::Clock> getClock() = 0;

  virtual rclcpp::Time now() = 0;
};
}  // namespace spot_ros2
