// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <memory>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>

namespace spot_ros2 {
RclcppNodeInterface::RclcppNodeInterface(
    const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>& node_base_interface)
    : node_base_interface_{node_base_interface} {}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> RclcppNodeInterface::getNodeBaseInterface() {
  return node_base_interface_;
}
}  // namespace spot_ros2
