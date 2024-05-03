// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>

#include <chrono>
#include <memory>

namespace spot_ros2 {
/**
 * @brief Implements TimerInterfaceBase to use rclcpp's WallTimer.
 */
class RclcppWallTimerInterface : public TimerInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppWallTimerInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppWallTimerInterface shares ownership of the shared_ptr.
   */
  explicit RclcppWallTimerInterface(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief The destructor
   */
  virtual ~RclcppWallTimerInterface();

  void setTimer(const std::chrono::duration<double>& period, const std::function<void()>& callback,
                const MultiThreading multithreading = MultiThreading::MutuallyExclusive) override;
  void clearTimer() override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::shared_ptr<rclcpp::CallbackGroup> callback_group_;
};
}  // namespace spot_ros2
