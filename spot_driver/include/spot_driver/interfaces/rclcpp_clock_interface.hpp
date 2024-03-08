// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <spot_driver/interfaces/clock_interface_base.hpp>

namespace spot_ros2 {
/**
 * @brief Implements TimerInterfaceBase to use rclcpp's WallTimer.
 */
class RclcppClockInterface : public ClockInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppWallTimerInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppWallTimerInterface shares ownership of the shared_ptr.
   */
  explicit RclcppClockInterface(
      const std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface>& node_clock_interface);

  rclcpp::Time now() override;

 private:
  std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface_;
};
}  // namespace spot_ros2
