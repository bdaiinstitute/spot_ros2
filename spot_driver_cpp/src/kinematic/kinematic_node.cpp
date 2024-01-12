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

KinematicNode::KinematicNode(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spotApi,
                             std::shared_ptr<ParameterInterfaceBase> parameterInterface,
                             const std::shared_ptr<LoggerInterfaceBase> loggerInterface) {
  initialize(node, std::move(spotApi), parameterInterface, loggerInterface);
}

KinematicNode::KinematicNode(const rclcpp::NodeOptions& nodeOptions) {
  auto node = std::make_shared<rclcpp::Node>("kinematic_service", nodeOptions);
  auto spotApi = std::make_unique<DefaultSpotApi>(kSDKClientName);
  auto parameterInterface = std::make_shared<RclcppParameterInterface>(node);
  auto loggerInterface = std::make_shared<RclcppLoggerInterface>(node->get_logger());
  initialize(node, std::move(spotApi), parameterInterface, loggerInterface);
}

void KinematicNode::initialize(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spotApi,
                               std::shared_ptr<ParameterInterfaceBase> parameterInterface,
                               const std::shared_ptr<LoggerInterfaceBase> loggerInterface) {
  node_ = node;
  spotApi_ = std::move(spotApi);

  const auto address = parameterInterface->getAddress();
  const auto robotName = parameterInterface->getSpotName();
  const auto username = parameterInterface->getUsername();
  const auto password = parameterInterface->getPassword();

  // create and authenticate robot
  if (const auto createRobotResult = spotApi_->createRobot(address, robotName); !createRobotResult) {
    const auto errorMsg{std::string{"Failed to create interface to robot: "}.append(createRobotResult.error())};
    loggerInterface->logError(errorMsg);
    throw std::runtime_error(errorMsg);
  }

  if (const auto authenticationResult = spotApi_->authenticate(username, password); !authenticationResult) {
    const auto errorMsg{std::string{"Failed to authenticate robot: "}.append(authenticationResult.error())};
    loggerInterface->logError(errorMsg);
    throw std::runtime_error(errorMsg);
  }

  internal_ = std::make_unique<KinematicService>(spotApi_->kinematicApi(), loggerInterface,
                                                 std::make_unique<KinematicMiddlewareHandle>(node_));
  internal_->initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> KinematicNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}
}  // namespace spot_ros2::kinematic
