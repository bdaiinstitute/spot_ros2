// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_api.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/spot_image_publisher_node.hpp>

namespace spot_ros2 {
SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options) {
  node_ = std::make_shared<rclcpp::Node>("image_publisher", node_options);
  spot_api = std::make_unique<DefaultSpotApi>();

  const auto logger = std::make_unique<RclcppLoggerInterface>(node_->get_logger());
  const auto parameters = std::make_unique<RclcppParameterInterface>(node_);

  const auto address = parameters->getAddress();
  const auto robot_name = parameters->getSpotName();
  const auto username = parameters->getUsername();
  const auto password = parameters->getPassword();

  // create and authenicate robot
  const auto create_robot_result = spot_api->createRobot(address, robot_name);
  if (!create_robot_result) {
    logger->logError(std::string{"Failed to create interface to robot: "}.append(create_robot_result.error()));
    throw;  // TODO(abaker): Define SpotApiError
  }
  const auto authentication_result = spot_api->authenicate(username, password);
  if (!authenication_result) {
    logger->logError(std::string{"Failed to authenicate robot: "}.append(authencation_result.error()));
    throw;  // TODO(abaker): Define SpotApiError
  }

  auto expected_image_client = spot_api->imageClient();
  if (!expected_image_client) {
    logger->logError(
        std::string{"Failed to retrieve image client from spot api: "}.append(expected_image_client.error()))
  }

  internal_ = std::make_unique<SpotImagePublisher>(node_, expected_image_client.value());
  internal_->initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}
}  // namespace spot_ros2
