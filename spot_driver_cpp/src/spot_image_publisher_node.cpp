// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_publisher_node.hpp>

#include <spot_driver_cpp/api/default_spot_api.hpp>
#include <spot_driver_cpp/images/spot_image_publisher.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>

namespace {
constexpr auto kSDKClientName = "spot_image_publisher";
}

namespace spot_ros2 {
SpotImagePublisherNode::SpotImagePublisherNode(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                                               const std::shared_ptr<ParameterInterfaceBase>& parameter_interface,
                                               const std::shared_ptr<LoggerInterfaceBase>& logger_interface)
    : node_{node}, spot_api_{std::move(spot_api)} {
  const auto address = parameter_interface->getAddress();
  const auto robot_name = parameter_interface->getSpotName();
  const auto username = parameter_interface->getUsername();
  const auto password = parameter_interface->getPassword();

  // create and authenticate robot
  if (const auto create_robot_result = spot_api_->createRobot(address, robot_name); !create_robot_result) {
    const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto authentication_result = spot_api_->authenticate(username, password); !authentication_result) {
    const auto error_msg{std::string{"Failed to authenticate robot: "}.append(authentication_result.error())};
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  const auto expected_has_arm = spot_api_->hasArm();
  if (!expected_has_arm) {
    const auto error_msg{std::string{"Failed to check availability of spot arm: "}.append(expected_has_arm.error())};
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ =
      std::make_unique<images::SpotImagePublisher>(node_, spot_api_->image_client_api(), expected_has_arm.value());
  internal_->initialize();
}

SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options) {
  auto node = std::make_shared<rclcpp::Node>("image_publisher", node_options);
  SpotImagePublisherNode(node, std::make_unique<DefaultSpotApi>(kSDKClientName),
                         std::make_shared<RclcppParameterInterface>(node),
                         std::make_shared<RclcppLoggerInterface>(node->get_logger()));
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}

}  // namespace spot_ros2
