// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
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
   * @brief Constructor for StatePublisherNode.
   * @details This constructor takes each interface as a pointer to the interface's base class, which enables dependency
   * inversion for improved testability.
   *
   * @param node_base_interface Exposes the NodeBaseInterface of this class's rclcpp::Node so this class can be spun by
   * an rclcpp executor.
   * @param spot_api Connects to Spot and exposes interfaces to request data from it.
   * @param middleware_handle Publishes robot state info to the middleware.
   * @param parameter_interface Retrieves runtime configuration settings needed to connect to and communicate with Spot.
   * @param logger_interface Logs error messages if requesting, processing, and publishing the robot state info does not
   * succeed.
   * @param tf_broadcaster_interface Publishes the dynamic transforms in Spot's robot state to TF.
   * @param timer_interface Repeatedly triggers timerCallback() using the middleware's clock.
   *
   */
  StatePublisherNode(std::unique_ptr<NodeInterfaceBase> node_base_interface, std::unique_ptr<SpotApi> spot_api,
                     std::unique_ptr<StatePublisher::MiddlewareHandle> middleware_handle,
                     std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                     std::unique_ptr<LoggerInterfaceBase> logger_interface,
                     std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                     std::unique_ptr<TimerInterfaceBase> timer_interface);

  /**
   * @brief Constructor for StatePublisherNode.
   * @details This constructor creates an rclcpp::Node and rclcpp-specialized implementations of the middleware
   * interfaces.
   *
   * @param node_options node configuration options used when creating a rclcpp::Node
   */
  explicit StatePublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning this class's node as if it were derived from rclcpp::Node.
   * This allows loading this class as a component node in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:
  /**
   * @brief Connect to and authenticate with Spot, and then create the StatePublisher class member.
   *
   * @param node_base_interface Exposes the NodeBaseInterface of this class's rclcpp::Node so this class can be spun by
   * an rclcpp executor.
   * @param spot_api Connects to Spot and exposes interfaces to request data from it.
   * @param middleware_handle Publishes robot state info to the middleware.
   * @param parameter_interface Retrieves runtime configuration settings needed to connect to and communicate with Spot.
   * @param logger_interface Logs error messages if requesting, processing, and publishing the robot state info does not
   * succeed.
   * @param tf_broadcaster_interface Publishes the dynamic transforms in Spot's robot state to TF.
   * @param timer_interface Repeatedly triggers timerCallback() using the middleware's clock.
   *
   * @throw std::runtime_error if the Spot API fails to create a connection to Spot or fails to authenticate with Spot.
   */
  void initialize(std::unique_ptr<SpotApi> spot_api,
                  std::unique_ptr<StatePublisher::MiddlewareHandle> middleware_handle,
                  std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                  std::unique_ptr<LoggerInterfaceBase> logger_interface,
                  std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                  std::unique_ptr<TimerInterfaceBase> timer_interface);

  std::unique_ptr<NodeInterfaceBase> node_base_interface_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<StatePublisher> internal_;
};
}  // namespace spot_ros2
