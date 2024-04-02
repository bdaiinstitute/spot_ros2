// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class to return a rclcpp NodeBaseInterface.
 */
class NodeInterfaceBase {
 public:
  // NodeInterfaceBase is move-only
  NodeInterfaceBase() = default;
  NodeInterfaceBase(NodeInterfaceBase&& other) = default;
  NodeInterfaceBase(const NodeInterfaceBase&) = delete;
  NodeInterfaceBase& operator=(NodeInterfaceBase&& other) = default;
  NodeInterfaceBase& operator=(const NodeInterfaceBase&) = delete;

  virtual ~NodeInterfaceBase() = default;

  virtual std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> getNodeBaseInterface() = 0;
};
}  // namespace spot_ros2
