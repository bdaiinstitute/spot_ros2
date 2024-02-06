// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/images/images_middleware_handle.hpp>
#include <spot_driver/images/spot_image_publisher.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>

namespace {
constexpr auto kSDKClientName = "spot_image_publisher";
}

namespace spot_ros2::images {
SpotImagePublisherNode::SpotImagePublisherNode(std::unique_ptr<SpotApi> spot_api,
                                               std::unique_ptr<SpotImagePublisher::MiddlewareHandle> mw_handle)
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

  const auto expected_has_arm = spot_api_->hasArm();
  if (!expected_has_arm) {
    const auto error_msg{std::string{"Failed to check availability of spot arm: "}.append(expected_has_arm.error())};
    mw_handle->logger_interface()->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ = std::make_unique<SpotImagePublisher>(spot_api_->image_client_interface(), std::move(mw_handle),
                                                   expected_has_arm.value());
  internal_->initialize();
}

SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options)
    : SpotImagePublisherNode{std::make_unique<DefaultSpotApi>(kSDKClientName),
                             std::make_unique<ImagesMiddlewareHandle>(node_options)} {}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}

}  // namespace spot_ros2::images
