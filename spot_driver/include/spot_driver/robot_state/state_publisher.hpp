// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <spot_driver/api/middleware_handle_base.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/types.hpp>

namespace spot_ros2 {

/**
 * @brief Retrieves robot state from Spot's robot state service client, converts the messages from Protobuf to ROS
 * messages, and publishes them to the appropriate topics
 */
class StatePublisher {
 public:
  /**
   * @brief A handle that enables dependency injection of ROS and rclcpp::Node operations
   */
  class MiddlewareHandle : public MiddlewareHandleBase {
   public:
    virtual ~MiddlewareHandle() = default;
    virtual void publishRobotState(const RobotStateMessages& robot_state_msgs) = 0;
  };

  /**
   * @brief Constructor for StatePublisher.
   * @details As opposed to other Spot Publishers, initialization takes place inside of the constructor because
   * initialization cannot fail.
   *
   * @param state_client_interface Requests robot state information from Spot through the Spot API.
   * @param time_sync_api  Converts message timestamps which are relative to Spot's clock to be relative to the host's
   * clock.
   * @param middleware_handle Publishes robot state info to the middleware.
   * @param parameter_interface Retrieves runtime-configurable settings, such as the preferred base frame.
   * @param logger_interface Logs error messages if requesting, processing, and publishing the robot state info does not
   * succeed.
   * @param tf_broadcaster_interface Publishes the dynamic transforms in Spot's robot state to TF.
   * @param timer_interface Repeatedly triggers timerCallback() using the middleware's clock.
   *
   */
  StatePublisher(const std::shared_ptr<StateClientInterface>& state_client_interface,
                 const std::shared_ptr<TimeSyncApi>& time_sync_api, std::unique_ptr<MiddlewareHandle> middleware_handle,
                 std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                 std::unique_ptr<LoggerInterfaceBase> logger_interface,
                 std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                 std::unique_ptr<TimerInterfaceBase> timer_interface);

 private:
  /**
   * @brief Callback function to retrieve and publish Spot's Robot State
   */
  void timerCallback();

  std::string full_tf_root_id_;

  std::string frame_prefix_;

  bool is_using_vision_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<StateClientInterface> state_client_interface_;
  std::shared_ptr<TimeSyncApi> time_sync_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
  std::unique_ptr<LoggerInterfaceBase> logger_interface_;
  std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface_;
  std::unique_ptr<TimerInterfaceBase> timer_interface_;
};
}  // namespace spot_ros2
