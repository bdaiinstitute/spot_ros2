// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver_cpp/api/spot_api.hpp>
#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>
#include <spot_driver_cpp/robot_state/spot_robot_state_publisher.hpp>

#include <memory>

namespace spot_ros2 {
/**
 * @brief Wraps SpotRobotStatePublisher to allow using it like a rclcpp::Node.
 */
class SpotRobotStatePublisherNode {
 public:
  /**
   * @brief Primary constructor for SpotRobotStatePublisherNode.
   * @details This constructor enables dependency inversion for improved testibility
   *
   * @param spot_api a unique_ptr of a SpotApi instance that this SpotRobotStatePublisherNode will take ownership of
   * @param mw_handle a unique_ptr of a SpotRobotStatePublisher::MiddlewareHandle instance that this
   * SpotRobotStatePublisherNode will take ownership of
   */
  SpotRobotStatePublisherNode(std::unique_ptr<SpotApi> spot_api,
                              std::unique_ptr<SpotRobotStatePublisher::MiddlewareHandle> mw_handle);

  /**
   * @brief Delegating constructor used in production.
   *
   * @param node_options node configuration options used when creating a rclcpp::Node
   */
  explicit SpotRobotStatePublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node.
   * This allows loading this class as a component node in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<SpotRobotStatePublisher> internal_;
};
}  // namespace spot_ros2