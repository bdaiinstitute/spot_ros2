// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/api/world_object_client_interface.hpp>
#include <spot_driver/interfaces/clock_interface_base.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/robot_model_interface_base.hpp>
#include <spot_driver/interfaces/tf_interface_base.hpp>
#include <spot_driver/interfaces/tf_listener_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/types.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
/**
 * @brief Monitors the kinematic tree managed by TF in ROS and the world model managed within Spot. For each TF frame
 * which is not associated with a link that is either internal to Spot (body and limb links) or managed by Spot
 * (AprilTags, docks, etc.), add a new WorldObject to Spot's internal world model to represent this object.
 *
 */
class ObjectSynchronizer {
 public:
  /**
   * @brief Constructor for ObjectSynchronizer
   *
   * @param world_object_client_interface Queries and modifies the objects in Spot's world model.
   * @param time_sync_api Gets clock skew measurements from Spot.
   * @param parameter_interface Retrieves runtime configuration settings needed to connect to and communicate with Spot.
   * @param logger_interface Logs info, warning, and error messages to the middleware.
   * @param tf_listener_interface Allows performing transform lookups between frames in the TF tree.
   * @param timer_interface Allows repeatedly requesting the lists of known world objects and known TF frame IDs.
   * @param clock_interface Gets the current timestamp when looking up transforms.
   */
  ObjectSynchronizer(const std::shared_ptr<WorldObjectClientInterface>& world_object_client_interface,
                     const std::shared_ptr<TimeSyncApi>& time_sync_api,
                     std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                     std::unique_ptr<LoggerInterfaceBase> logger_interface,
                     std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                     std::unique_ptr<TimerInterfaceBase> timer_interface,
                     std::unique_ptr<ClockInterfaceBase> clock_interface);

 private:
  /**
   * @brief Timer callback function called by timer_interface_.
   * @details All the work of retrieving TF and world object information and updating world objects as needed happens
   * within this function.
   */
  void syncWorldObjects();

  std::string frame_prefix_;
  std::string preferred_base_frame_;
  std::string preferred_base_frame_with_prefix_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<WorldObjectClientInterface> world_object_client_interface_;
  std::shared_ptr<TimeSyncApi> time_sync_interface_;
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
  std::unique_ptr<LoggerInterfaceBase> logger_interface_;
  std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface_;
  std::unique_ptr<TimerInterfaceBase> timer_interface_;
  std::unique_ptr<ClockInterfaceBase> clock_interface_;
};

}  // namespace spot_ros2
