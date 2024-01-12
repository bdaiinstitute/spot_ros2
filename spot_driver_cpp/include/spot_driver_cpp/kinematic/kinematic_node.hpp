// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/kinematic/kinematic_service.hpp>

#include <spot_driver_cpp/api/spot_api.hpp>
#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>

#include <memory>

namespace spot_ros2::kinematic {
class KinematicNode {
 public:
  explicit KinematicNode(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spotApi,
                         std::shared_ptr<ParameterInterfaceBase> parameterInterface,
                         std::shared_ptr<LoggerInterfaceBase> loggerInterface);

  explicit KinematicNode(const rclcpp::NodeOptions& nodeOptions = rclcpp::NodeOptions{});

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
  std::unique_ptr<SpotApi> spotApi_;
  std::unique_ptr<KinematicService> internal_;

  void initialize(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spotApi,
                  std::shared_ptr<ParameterInterfaceBase> parameterInterface,
                  const std::shared_ptr<LoggerInterfaceBase> loggerInterface);
};
}  // namespace spot_ros2::kinematic
