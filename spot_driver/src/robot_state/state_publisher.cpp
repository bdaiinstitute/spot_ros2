// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <rclcpp/node.hpp>
#include <spot_driver/api/default_robot_state_client.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/robot_state/state_publisher.hpp>
#include <utility>

namespace {
constexpr auto kRobotStateCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0};  // 50 Hz
}

namespace spot_ros2 {

StatePublisher::StatePublisher(std::shared_ptr<RobotStateClientInterface> robot_state_client_interface,
                               std::unique_ptr<MiddlewareHandle> middleware_handle,
                               std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                               std::unique_ptr<LoggerInterfaceBase> logger_interface,
                               std::unique_ptr<TfInterfaceBase> tf_interface,
                               std::unique_ptr<TimerInterfaceBase> timer_interface)
    : client_interface_{std::move(robot_state_client_interface)},
      middleware_handle_{std::move(middleware_handle)},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      tf_interface_{std::move(tf_interface)},
      timer_interface_{std::move(timer_interface)} {
  const auto preferred_odom_frame = parameter_interface_->getPreferredOdomFrame();

  full_odom_frame_id_ = preferred_odom_frame.find('/') == std::string::npos
                            ? parameter_interface_->getSpotName() + "/" + preferred_odom_frame
                            : preferred_odom_frame;

  // Create a timer to request and publish robot state at a fixed rate
  timer_interface_->setTimer(kRobotStateCallbackPeriod, [this]() {
    timerCallback();
  });
}

void StatePublisher::timerCallback() {
  const auto robot_state_result = client_interface_->getRobotState(full_odom_frame_id_);
  if (!robot_state_result.has_value()) {
    logger_interface_->logError(std::string{"Failed to get robot_state: "}.append(robot_state_result.error()));
    return;
  }

  middleware_handle_->publishRobotState(robot_state_result.value());

  if (robot_state_result->maybe_tf) {
    tf_interface_->sendDynamicTransforms(robot_state_result->maybe_tf->transforms);
  }
}

}  // namespace spot_ros2
