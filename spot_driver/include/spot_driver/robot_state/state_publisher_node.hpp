// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/robot_state/state_publisher.hpp>

#include <memory>
#include "spot_driver/interfaces/node_interface_base.hpp"

namespace spot_ros2 {
/**
 * @brief Wraps StatePublisher to allow using it like a rclcpp::Node.
 */
class StatePublisherNode {
 public:
  /**
   * @brief Primary constructor for StatePublisherNode.
   * @details This constructor enables dependency inversion for improved testibility
   *
   * @param spot_api a unique_ptr of a SpotApi instance that this StatePublisherNode will take ownership of
   * @param mw_handle a unique_ptr of a StatePublisher::MiddlewareHandle instance that this
   * StatePublisherNode will take ownership of
   */
  StatePublisherNode(std::unique_ptr<NodeInterfaceBase> node_base_interface, std::unique_ptr<SpotApi> spot_api,
                     std::unique_ptr<StatePublisher::MiddlewareHandle> mw_handle,
                     std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                     std::unique_ptr<LoggerInterfaceBase> logger_interface,
                     std::unique_ptr<TfInterfaceBase> tf_interface,
                     std::unique_ptr<TimerInterfaceBase> timer_interface);

  /**
   * @brief Delegating constructor used in production.
   *
   * @param node_options node configuration options used when creating a rclcpp::Node
   */
  explicit StatePublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node.
   * This allows loading this class as a component node in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:
  void initialize(std::unique_ptr<SpotApi> spot_api, std::unique_ptr<StatePublisher::MiddlewareHandle> mw_handle,
                  std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                  std::unique_ptr<LoggerInterfaceBase> logger_interface, std::unique_ptr<TfInterfaceBase> tf_interface,
                  std::unique_ptr<TimerInterfaceBase> timer_interface);

  std::unique_ptr<NodeInterfaceBase> node_base_interface_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<StatePublisher> internal_;
};
}  // namespace spot_ros2
