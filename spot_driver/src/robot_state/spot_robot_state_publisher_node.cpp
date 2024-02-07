// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/robot_state/spot_robot_state_publisher.hpp>
#include <spot_driver/robot_state/spot_robot_state_publisher_node.hpp>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/robot_state/spot_robot_state_middleware_handle.hpp>

namespace {
constexpr auto kDefaultSDKName{"robot_state_publisher_node"};
}

namespace spot_ros2 {

SpotRobotStatePublisherNode::SpotRobotStatePublisherNode(
    std::unique_ptr<SpotApi> spot_api, std::unique_ptr<SpotRobotStatePublisher::MiddlewareHandle> mw_handle)
    : node_{mw_handle->node()}, spot_api_{std::move(spot_api)} {
  const auto address = mw_handle->parameter_interface()->getAddress();
  const auto robot_name = mw_handle->parameter_interface()->getSpotName();
  const auto username = mw_handle->parameter_interface()->getUsername();
  const auto password = mw_handle->parameter_interface()->getPassword();

  // create and authenticate robot
  if (const auto create_robot_result = spot_api_->createRobot(address, robot_name); !create_robot_result) {
    const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
    mw_handle->logger_interface()->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto authentication_result = spot_api_->authenticate(username, password); !authentication_result) {
    const auto error_msg{std::string{"Failed to authenticate robot: "}.append(authentication_result.error())};
    mw_handle->logger_interface()->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ =
      std::make_unique<SpotRobotStatePublisher>(spot_api_->robot_state_client_interface(), std::move(mw_handle));
}

SpotRobotStatePublisherNode::SpotRobotStatePublisherNode(const rclcpp::NodeOptions& node_options)
    : SpotRobotStatePublisherNode{std::make_unique<DefaultSpotApi>(kDefaultSDKName),
                                  std::make_unique<SpotRobotStateMiddlewareHandle>(node_options)} {}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotRobotStatePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}

}  // namespace spot_ros2
