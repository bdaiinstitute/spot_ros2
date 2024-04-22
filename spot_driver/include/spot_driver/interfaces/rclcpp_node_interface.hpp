// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <spot_driver/interfaces/node_interface_base.hpp>

namespace spot_ros2 {
/**
 * @brief Implementation of NodeInterfaceBase that provides the node base interface of a rclcpp Node.
 */
class RclcppNodeInterface final : public NodeInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppNodeInterface.
   * @param node_base_interface A shared_ptr to the NodeBaseInterface of a rclcpp Node. The RclcppNodeInterface class
   * will share ownership of this pointer through its node_base_interface_ member.
   */
  explicit RclcppNodeInterface(const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>& node_base_interface);

  [[nodiscard]] std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> getNodeBaseInterface() override;

 private:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface_;
};
}  // namespace spot_ros2
