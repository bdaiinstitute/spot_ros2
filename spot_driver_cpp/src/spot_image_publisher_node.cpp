// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_publisher_node.hpp>

#include <spot_driver_cpp/api/default_spot_api.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_middleware_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/spot_image_publisher.hpp>

namespace {
constexpr auto kSDKClientName = "spot_image_publisher";
}

namespace spot_ros2 {
SpotImagePublisherNode::SpotImagePublisherNode(std::shared_ptr<rclcpp::Node> node,
                                               std::shared_ptr<MiddlewareInterface> middleware_interface)
    : node_{node}, middleware_interface_{middleware_interface} {
  // TODO(abaker): Mock out spot_api_
  spot_api_ = std::make_unique<DefaultSpotApi>(kSDKClientName);

  const auto logger = middleware_interface_->logger_interface();
  const auto parameters = middleware_interface_->parameter_interface();

  const auto address = parameters->getAddress();
  const auto robot_name = parameters->getSpotName();
  const auto username = parameters->getUsername();
  const auto password = parameters->getPassword();

  // create and authenticate robot
  const auto create_robot_result = spot_api_->createRobot(address, robot_name);
  if (!create_robot_result) {
    logger->logError(std::string{"Failed to create interface to robot: "}.append(create_robot_result.error()));
    throw;  // TODO(abaker): Define SpotApiException
  }
  const auto authentication_result = spot_api_->authenticate(username, password);
  if (!authentication_result) {
    logger->logError(std::string{"Failed to authenticate robot: "}.append(authentication_result.error()));
    throw;  // TODO(abaker): Define SpotApiException
  }

  auto expected_image_client = spot_api_->imageClient();
  if (!expected_image_client) {
    logger->logError(
        std::string{"Failed to retrieve image client from spot api: "}.append(expected_image_client.error()));
    throw;  // TODO(abaker): Define SpotApiException
  }

  const auto expected_has_arm = spot_api_->hasArm();
  if (!expected_has_arm) {
    logger->logError(std::string{"Failed to check availability of spot arm: "}.append(expected_has_arm.error()));
    throw;  // TODO(abaker): Define SpotApiException
  }

  internal_ =
      std::make_unique<SpotImagePublisher>(node_, std::move(expected_image_client.value()), expected_has_arm.value());
  internal_->initialize();
}

SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options) {
  auto node = std::make_shared<rclcpp::Node>("image_publisher", node_options);
  SpotImagePublisherNode(node, std::make_shared<RclcppMiddlewareInterface>(node));
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}

}  // namespace spot_ros2
