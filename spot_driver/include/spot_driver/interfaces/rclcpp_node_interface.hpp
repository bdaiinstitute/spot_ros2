// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <spot_driver/interfaces/node_interface_base.hpp>

namespace spot_ros2 {
/**
 * @brief Implementation of LoggerInterfaceBase that logs messages using rclcpp's logging utilities.
 */
class RclcppNodeInterface : public NodeInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppNodeInterface.
   * @param logger An instance of a logger which will be used to generate ROS 2 logs. This will be copied into the
   * logger_ member.
   */
  explicit RclcppNodeInterface(const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>& node_base_interface);

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> getNodeBaseInterface() override;

 private:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface_;
};
}  // namespace spot_ros2
