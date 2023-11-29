// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/kinematic_node.hpp>

#include <spot_driver_cpp/api/default_kinematic_api.hpp>
#include <spot_driver_cpp/api/default_robot_api.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>

#include <memory>

namespace spot_ros2 {
spot_ros2::KinematicNode::KinematicNode(const rclcpp::NodeOptions& node_options) {
  node_ = std::make_shared<rclcpp::Node>("image_publisher", node_options);

  auto logger = std::make_unique<RclcppLoggerInterface>(node_->get_logger());

  auto parameters = std::make_unique<RclcppParameterInterface>(node_);
  const auto address = parameters->getAddress();
  const auto robot_name = parameters->getSpotName();
  const auto username = parameters->getUsername();
  const auto password = parameters->getPassword();

  // Get a robot.
  std::shared_ptr<Robot> robot;
  auto robot_api = std::make_unique<DefaultRobotApi>();
  if (auto result = robot_api->createRobot(address, robot_name); !result) {
    logger->logError(std::string{"Failed to create interface to robot: "}.append(result.error()));
    throw;
  } else {
    robot = std::shared_ptr<Robot>{std::move(result.value())};
  }

  // Authenticate.
  if (auto result = robot->authenticate(username, password); !result) {
    logger->logError("Authentication with provided username and password did not succeed.");
    throw;
  }

  // Create the required API.
  auto kinematic_api = std::make_unique<DefaultKinematicApi>(robot);
  if (auto result = kinematic_api->init(); !result) {
    logger->logError(std::string{"Error initializing api: "}.append(result.error()));
    throw;
  }

  kinematic_service_ = std::make_unique<KinematicService>(node_, std::move(kinematic_api));
  kinematic_service_->init();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> spot_ros2::KinematicNode::get_node_base_interface() {
  return std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>();
}
}  // namespace spot_ros2
