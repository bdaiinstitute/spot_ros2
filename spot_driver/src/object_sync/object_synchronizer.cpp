// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/types.hpp>
#include <utility>

namespace {
constexpr auto kTfSyncPeriod = std::chrono::duration<double>{1.0 / 1.0};  // 1 Hz
}

namespace spot_ros2 {

ObjectSynchronizer::ObjectSynchronizer(const std::shared_ptr<StateClientInterface>& state_client_interface,
                                       const std::shared_ptr<WorldObjectClientInterface>& world_object_client_interface,
                                       const std::shared_ptr<TimeSyncApi>& time_sync_api,
                                       std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                       std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                       std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                                       std::unique_ptr<TimerInterfaceBase> timer_interface)
    : state_client_interface_{state_client_interface},
      world_object_client_interface_{world_object_client_interface},
      logger_interface_{std::move(logger_interface)},
      tf_listener_interface_{std::move(tf_listener_interface)},
      timer_interface_{std::move(timer_interface)} {
  timer_interface_->setTimer(kTfSyncPeriod, [this]() {
    onTimer();
  });
}

void ObjectSynchronizer::onTimer() {
  // Get all TF frame IDs
  const auto tf_frame_names = tf_listener_interface_->getAllFrameNames();

  // Get all frame IDs which are a part of Spot (I think this is fixed once we set whether Spot has an arm. Can retrieve
  // from the URDF published by RobotStatePublisher.) Get all the frame IDs which are published by Spot but cannot be
  // modified by this node (for example, AR tag poses)

  ::bosdyn::api::ListWorldObjectRequest request_non_mutable_frames;
  request_non_mutable_frames.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_APRILTAG);
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto non_mutable_frames_response = world_object_client_interface_->listWorldObjects(request_non_mutable_frames);
  if (!non_mutable_frames_response) {
    logger_interface_->logError("Failed to list world objects.");
    return;
  }
  // Each WorldObject has its own transform snapshot
  // non_mutable_frames_response.value().world_objects().at(0)

  // If a TF frame does not originate from Spot:
  //   If the frame is already included in the world objects:
  //     Mutate world objects to update the transform of the frame's object
  //   Else (frame is not a world object):
  //     Mutate world objects to add the frame as a new object

  // Each frame is represented by a WorldObject which contains a single frame.
  // The name of the object is identical to the frame ID.

  ::bosdyn::api::ListWorldObjectRequest request;
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_UNKNOWN);
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  world_object_client_interface_->listWorldObjects(request);
}

}  // namespace spot_ros2
