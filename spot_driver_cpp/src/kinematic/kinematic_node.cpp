// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/kinematic_middleware_handle.hpp>
#include <spot_driver_cpp/kinematic/kinematic_node.hpp>

#include <spot_driver_cpp/api/default_spot_api.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>

#include <memory>

namespace {
constexpr auto kSDKClientName = "inverse_kinematic";
}

namespace spot_ros2::kinematic {

KinematicNode::KinematicNode(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                             std::shared_ptr<ParameterInterfaceBase> parameter_interface,
                             const std::shared_ptr<LoggerInterfaceBase> logger_interface) {
  initialize(node, std::move(spot_api), parameter_interface, logger_interface);
}

KinematicNode::KinematicNode(const rclcpp::NodeOptions& node_options) {
  auto node = std::make_shared<rclcpp::Node>("kinematic_service", node_options);
  auto spot_api = std::make_unique<DefaultSpotApi>(kSDKClientName);
  auto parameter_interface = std::make_shared<RclcppParameterInterface>(node);
  auto logger_interface = std::make_shared<RclcppLoggerInterface>(node->get_logger());
  initialize(node, std::move(spot_api), parameter_interface, logger_interface);
}

void KinematicNode::initialize(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                               std::shared_ptr<ParameterInterfaceBase> parameter_interface,
                               const std::shared_ptr<LoggerInterfaceBase> logger_interface) {
  node_ = node;
  spot_api_ = std::move(spot_api);

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

  if (const auto authenticationResult = spot_api_->authenticate(username, password); !authenticationResult) {
    const auto errorMsg{std::string{"Failed to authenticate robot: "}.append(authenticationResult.error())};
    logger_interface->logError(errorMsg);
    throw std::runtime_error(errorMsg);
  }

  internal_ = std::make_unique<KinematicService>(spot_api_->kinematicApi(), logger_interface,
                                                 std::make_unique<KinematicMiddlewareHandle>(node_));
  internal_->initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> KinematicNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}
}  // namespace spot_ros2::kinematic
