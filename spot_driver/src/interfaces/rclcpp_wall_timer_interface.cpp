// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace spot_ros2 {
RclcppWallTimerInterface::RclcppWallTimerInterface(const std::shared_ptr<rclcpp::Node>& node) : node_{node} {}

RclcppWallTimerInterface::~RclcppWallTimerInterface() {
  clearTimer();
}

void RclcppWallTimerInterface::setTimer(const std::chrono::duration<double>& period,
                                        const std::function<void()>& callback, const MultiThreading multithreading) {
  if (multithreading == MultiThreading::MutuallyExclusive) {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  } else {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  timer_ = node_->create_wall_timer(period, callback, callback_group_);
}

void RclcppWallTimerInterface::clearTimer() {
  timer_.reset();
  callback_group_.reset();
}
}  // namespace spot_ros2
