// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_state_client.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver_cpp/robot_state/spot_robot_state_publisher.hpp>

#include <chrono>
#include <rclcpp/node.hpp>

namespace {
constexpr auto kRobotStateCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0};  // 50 Hz
}

namespace spot_ros2 {

SpotRobotStatePublisher::SpotRobotStatePublisher(
    std::shared_ptr<RobotStateClientInterface> robot_state_client_interface,
    std::unique_ptr<MiddlewareHandle> middleware_handle)
    : client_interface_{robot_state_client_interface}, middleware_handle_{std::move(middleware_handle)} {}

bool SpotRobotStatePublisher::initialize() {
  // Create a publisher for all messages in robot state
  middleware_handle_->createPublishers();

  // Create a timer to request and publish robot state at a fixed rate
  middleware_handle_->timer_interface()->setTimer(kRobotStateCallbackPeriod, [this]() {
    timerCallback();
  });

  return true;
}

void SpotRobotStatePublisher::timerCallback() {
  const auto preferred_odom_frame = middleware_handle_->parameter_interface()->getPreferredOdomFrame().find("/") == std::string::npos ?
     middleware_handle_->parameter_interface()->getSpotName() + "/" + middleware_handle_->parameter_interface()->getPreferredOdomFrame() :
     middleware_handle_->parameter_interface()->getPreferredOdomFrame(); 

  const auto robot_state_result = client_interface_->getRobotState(preferred_odom_frame);
  if (!robot_state_result.has_value()) {
    middleware_handle_->logger_interface()->logError(
        std::string{"Failed to get robot_state: "}.append(robot_state_result.error()));
    return;
  }

  middleware_handle_->publishRobotState(robot_state_result.value());
}

}  // namespace spot_ros2
