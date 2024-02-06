// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace spot_ros2 {
RclcppWallTimerInterface::RclcppWallTimerInterface(const std::shared_ptr<rclcpp::Node>& node) : node_{node} {}

void RclcppWallTimerInterface::setTimer(const std::chrono::duration<double>& period,
                                        const std::function<void()>& callback) {
  timer_ = node_->create_wall_timer(period, callback);
}

void RclcppWallTimerInterface::clearTimer() {
  timer_.reset();
}
}  // namespace spot_ros2
