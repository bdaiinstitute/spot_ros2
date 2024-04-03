// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/interfaces/rclcpp_clock_interface.hpp>

namespace spot_ros2 {
RclcppClockInterface::RclcppClockInterface(
    const std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface>& node_clock_interface)
    : node_clock_interface_{node_clock_interface} {}

rclcpp::Time RclcppClockInterface::now() {
  return node_clock_interface_->get_clock()->now();
}

}  // namespace spot_ros2
