// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_api.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/spot_image_publisher_node.hpp>

namespace spot_ros2 {
SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options) {
  node_ = std::make_shared<rclcpp::Node>("image_publisher", node_options);

  auto logger = std::make_unique<RclcppLoggerInterface>(node_->get_logger());

  auto parameters = std::make_unique<RclcppParameterInterface>(node_);
  const auto address = parameters->getAddress();
  const auto robot_name = parameters->getSpotName();
  const auto username = parameters->getUsername();
  const auto password = parameters->getPassword();

  // Get a robot.
  auto robot_api = std::make_unique<DefaultRobotApi>();
  auto expected_robot = robot_api->createRobot(address, robot_name);
  if (!expected_robot) {
    logger->logError(std::string{"Failed to create interface to robot: "}.append(expected_robot.error()));
    exit(1);
  }

  // Authenticate.
  std::shared_ptr<Robot> robot{std::move(expected_robot.value())};
  auto auth = robot->authenticate(username, password);
  if (!auth) {
    logger->logError("Authentication with provided username and password did not succeed.");
    exit(1);
  }

  internal_ = std::make_unique<SpotImagePublisher>(robot, node_);
  internal_->initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}
}  // namespace spot_ros2
