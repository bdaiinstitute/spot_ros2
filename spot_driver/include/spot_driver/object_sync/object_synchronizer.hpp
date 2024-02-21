// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/api/world_object_client_interface.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/robot_model_interface_base.hpp>
#include <spot_driver/interfaces/tf_interface_base.hpp>
#include <spot_driver/interfaces/tf_listener_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/types.hpp>

#include <rclcpp/node.hpp>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
class ObjectSynchronizer {
 public:
  ObjectSynchronizer(const std::shared_ptr<StateClientInterface>& state_client_interface,
                     const std::shared_ptr<WorldObjectClientInterface>& world_object_client_interface,
                     const std::shared_ptr<TimeSyncApi>& time_sync_api,
                     std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                     std::unique_ptr<LoggerInterfaceBase> logger_interface,
                     std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                     std::unique_ptr<TimerInterfaceBase> timer_interface,
                     std::unique_ptr<RobotModelInterfaceBase> robot_model_interface);

 private:
  void onTimer();

  std::string full_odom_frame_id_;

  std::string frame_prefix_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<StateClientInterface> state_client_interface_;

  std::shared_ptr<WorldObjectClientInterface> world_object_client_interface_;

  std::shared_ptr<TimeSyncApi> time_sync_interface_;

  /** @brief instance of ParameterInterfaceBase to get ROS parameters*/
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
  /** @brief instance of LoggerInterfaceBase to send ROS log messages*/
  std::unique_ptr<LoggerInterfaceBase> logger_interface_;

  std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface_;
  /** @brief instance of TimerInterfaceBase to create callback timer*/
  std::unique_ptr<TimerInterfaceBase> timer_interface_;

  std::unique_ptr<RobotModelInterfaceBase> robot_model_interface_;
};

}  // namespace spot_ros2
