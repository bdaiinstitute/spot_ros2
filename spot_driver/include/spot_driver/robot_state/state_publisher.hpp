// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/tf_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/types.hpp>

#include <rclcpp/node.hpp>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

/**
 * @brief Retrieves robot state from Spot's robot state service client, converts the messages from Protobuf to ROS
 * messages, and publishes them to the appropriate topics
 */
class StatePublisher {
 public:
  /**
   * @brief A handle that enables dependency injection of ROS and rclcpp::node operations
   */
  class MiddlewareHandle {
   public:
    virtual ~MiddlewareHandle() = default;
    virtual void publishRobotState(const RobotStateMessages& robot_state_msgs) = 0;
  };

  /**
   * @brief Constructor for StatePublisher.
   * @details As opposed to other Spot Publishers, initialization takes place inside of the constructor because
   * initialization cannot fail.
   *
   * @param state_client_interface A shared_ptr to a StateClientInterface used to request robot state information from
   * Spot through the Spot API.
   * @param time_sync_api  A shared_ptr to a TymeSyncApi used to convert message timestamps which are relative to Spot's
   * clock to be relative to the host's clock.
   * @param middleware_handle A unique_ptr to an instance of a class derived from StatePublisher::MiddlewareHandle which
   * will be used to publish robot state messages.
   * @param logger_interface A unique_ptr to an instance of a class derived from NodeInterfaceBase which will be used to
   * log info and error messages.
   * @param tf_interface A unique_ptr to an instance of a class derived from TfInterfaceBase which will be used to
   * publish the transforms contained in Spot's robot state.
   * @param timer_interface A unique_ptr to an instance of a class derived from TimerInterfaceBase which will be used to
   * repeatedly trigger requests for updated robot states.
   *
   */
  StatePublisher(const std::shared_ptr<StateClientInterface>& state_client_interface,
                 const std::shared_ptr<TimeSyncApi>& time_sync_api, std::unique_ptr<MiddlewareHandle> middleware_handle,
                 std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                 std::unique_ptr<LoggerInterfaceBase> logger_interface, std::unique_ptr<TfInterfaceBase> tf_interface,
                 std::unique_ptr<TimerInterfaceBase> timer_interface);

 private:
  /**
   * @brief Callback function to retrieve and publish Spot's Robot State
   */
  void timerCallback();

  std::string full_odom_frame_id_;

  std::string frame_prefix_;

  bool is_using_vision_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<StateClientInterface> state_client_interface_;
  std::shared_ptr<TimeSyncApi> time_sync_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;

  /** @brief instance of ParameterInterfaceBase to get ROS parameters*/
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
  /** @brief instance of LoggerInterfaceBase to send ROS log messages*/
  std::unique_ptr<LoggerInterfaceBase> logger_interface_;
  /** @brief instance of TfInterfaceBase to update static transforms*/
  std::unique_ptr<TfInterfaceBase> tf_interface_;
  /** @brief instance of TimerInterfaceBase to create callback timer*/
  std::unique_ptr<TimerInterfaceBase> timer_interface_;
};

}  // namespace spot_ros2
