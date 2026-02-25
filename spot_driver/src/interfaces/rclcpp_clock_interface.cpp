// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <spot_driver/interfaces/rclcpp_clock_interface.hpp>

namespace spot_ros2 {
RclcppClockInterface::RclcppClockInterface(
    const std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface>& node_clock_interface)
    : node_clock_interface_{node_clock_interface} {}

rclcpp::Clock::SharedPtr RclcppClockInterface::getClock() {
  return node_clock_interface_->get_clock();
}

rclcpp::Time RclcppClockInterface::now() {
  return node_clock_interface_->get_clock()->now();
}

}  // namespace spot_ros2
