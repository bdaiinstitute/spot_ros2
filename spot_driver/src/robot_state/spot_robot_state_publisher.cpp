// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <rclcpp/node.hpp>
#include <spot_driver/api/default_robot_state_client.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/robot_state/spot_robot_state_publisher.hpp>
#include <utility>

namespace {
constexpr auto kRobotStateCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0};  // 50 Hz
}

namespace spot_ros2 {

SpotRobotStatePublisher::SpotRobotStatePublisher(
    std::shared_ptr<RobotStateClientInterface> robot_state_client_interface,
    std::unique_ptr<MiddlewareHandle> middleware_handle)
    : client_interface_{std::move(robot_state_client_interface)}, middleware_handle_{std::move(middleware_handle)} {
  // Create a publisher for all messages in robot state
  middleware_handle_->createPublishers();

  const auto preferred_odom_frame = middleware_handle_->parameter_interface()->getPreferredOdomFrame();

  full_odom_frame_id_ = preferred_odom_frame.find('/') == std::string::npos
                            ? middleware_handle_->parameter_interface()->getSpotName() + "/" + preferred_odom_frame
                            : preferred_odom_frame;

  // Create a timer to request and publish robot state at a fixed rate
  middleware_handle_->timer_interface()->setTimer(kRobotStateCallbackPeriod, [this]() {
    timerCallback();
  });
}

void SpotRobotStatePublisher::timerCallback() {
  const auto robot_state_result = client_interface_->getRobotState(full_odom_frame_id_);
  if (!robot_state_result.has_value()) {
    middleware_handle_->logger_interface()->logError(
        std::string{"Failed to get robot_state: "}.append(robot_state_result.error()));
    return;
  }

  middleware_handle_->publishRobotState(robot_state_result.value());
}

}  // namespace spot_ros2
