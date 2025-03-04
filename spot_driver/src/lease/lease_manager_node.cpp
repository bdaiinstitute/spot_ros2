// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/lease/lease_manager_node.hpp>
#include <spot_driver/lease/lease_middleware_handle.hpp>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>

#include <memory>
#include <stdexcept>

namespace spot_ros2::lease {

LeaseManagerNode::LeaseManagerNode(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                                   std::shared_ptr<ParameterInterfaceBase> parameter_interface,
                                   const std::shared_ptr<LoggerInterfaceBase> logger_interface) {
  initialize(node, std::move(spot_api), parameter_interface, logger_interface);
}

LeaseManagerNode::LeaseManagerNode(const rclcpp::NodeOptions& node_options) {
  auto node = std::make_shared<rclcpp::Node>("lease_manager", node_options);
  auto parameter_interface = std::make_shared<RclcppParameterInterface>(node);
  auto logger_interface = std::make_shared<RclcppLoggerInterface>(node->get_logger());
  const auto timesync_timeout = parameter_interface->getTimeSyncTimeout();
  auto spot_api =
      std::make_unique<DefaultSpotApi>("lease_manager", timesync_timeout, parameter_interface->getCertificate());
  initialize(node, std::move(spot_api), parameter_interface, logger_interface);
}

void LeaseManagerNode::initialize(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                                  std::shared_ptr<ParameterInterfaceBase> parameter_interface,
                                  const std::shared_ptr<LoggerInterfaceBase> logger_interface) {
  node_ = node;
  spot_api_ = std::move(spot_api);

  const auto hostname = parameter_interface->getHostname();
  const auto port = parameter_interface->getPort();
  const auto robot_name = parameter_interface->getSpotName();
  const auto username = parameter_interface->getUsername();
  const auto password = parameter_interface->getPassword();

  // create and authenticate robot
  if (const auto create_robot_result = spot_api_->createRobot(robot_name, hostname, port); !create_robot_result) {
    const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto authenticationResult = spot_api_->authenticate(username, password); !authenticationResult) {
    const auto errorMsg{std::string{"Failed to authenticate robot: "}.append(authenticationResult.error())};
    logger_interface->logError(errorMsg);
    throw std::runtime_error(errorMsg);
  }

  auto lease_client = spot_api_->leaseClientInterface();
  if (!lease_client) {
    constexpr auto errorMsg{"Failed to initialize the Spot API's lease client, which is required to run this node."};
    logger_interface->logError(errorMsg);
    throw std::runtime_error(errorMsg);
  }

  internal_ = std::make_unique<LeaseManager>(std::move(lease_client), logger_interface,
                                             std::make_unique<LeaseMiddlewareHandle>(node_));
  internal_->initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> LeaseManagerNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}
}  // namespace spot_ros2::lease
