// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <rclcpp/duration.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/conversions/common_conversions.hpp>
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
                                       std::unique_ptr<TimerInterfaceBase> timer_interface,
                                       std::unique_ptr<RobotModelInterfaceBase> robot_model_interface)
    : state_client_interface_{state_client_interface},
      world_object_client_interface_{world_object_client_interface},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      tf_listener_interface_{std::move(tf_listener_interface)},
      timer_interface_{std::move(timer_interface)},
      robot_model_interface_{std::move(robot_model_interface)} {
  const auto spot_name = parameter_interface_->getSpotName();
  frame_prefix_ = spot_name.empty() ? "" : spot_name + "/";
  timer_interface_->setTimer(kTfSyncPeriod, [this]() {
    onTimer();
  });
}

void ObjectSynchronizer::onTimer() {
  if (!world_object_client_interface_) {
    logger_interface_->logError("World object interface not initialized.");
    return;
  }

  // Get all TF frame IDs
  const auto tf_frame_names = tf_listener_interface_->getAllFrameNames();
  std::cout << "TF frames:\n";
  for (const auto& frame : tf_frame_names) {
    std::cout << "  " << frame << "\n";
  }
  std::cout << std::endl;

  // Get all frame IDs which are a part of Spot (I think this is fixed once we set whether Spot has an arm. Can retrieve
  // from the URDF published by RobotStatePublisher.) Get all the frame IDs which are published by Spot but cannot be
  // modified by this node (for example, AR tag poses)
  const auto robot_model_frames = robot_model_interface_->getFrameIds();
  if (!robot_model_frames) {
    logger_interface_->logError("Failed to get Spot frame IDs from robot model: " + robot_model_frames.error());
    return;
  }

  std::cout << "URDF frames:\n";
  for (const auto& frame : robot_model_frames.value()) {
    std::cout << "  " << frame << "\n";
  }
  std::cout << std::endl;

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

  std::set<std::string> world_object_names;
  std::cout << "world objects:\n";
  for (const auto& world_object : non_mutable_frames_response.value().world_objects()) {
    std::cout << "  " << world_object.name() << "\n";
    world_object_names.insert(world_object.name());
  }
  std::cout << std::endl;

  // If a TF frame does not originate from Spot:
  //   If the frame is already included in the world objects:
  //     Mutate world objects to update the transform of the frame's object
  //   Else (frame is not a world object):
  //     Mutate world objects to add the frame as a new object

  for (const auto& tf_frame : tf_frame_names) {
    if (robot_model_frames->count(tf_frame) > 0) {
      continue;
    }

    const auto tform_base_to_frame = tf_listener_interface_->lookupTransform(
        frame_prefix_ + "odom", tf_frame, rclcpp::Time{0, 0}, rclcpp::Duration{std::chrono::nanoseconds{0}});
    if (!tform_base_to_frame) {
      logger_interface_->logWarn(tform_base_to_frame.error());
      continue;
    }

    // Each frame is represented by a WorldObject which contains a single frame.
    // The name of the object is identical to the frame ID.
    // The world object client can only mutate a single world object at a time, so we send separate requests for each
    // object
    ::bosdyn::api::MutateWorldObjectRequest request;
    auto* object = request.mutable_mutation()->mutable_object();
    *object->mutable_name() = tf_frame;
    auto* edge_map = object->mutable_transforms_snapshot()->mutable_child_to_parent_edge_map();

    ::bosdyn::api::FrameTreeSnapshot_ParentEdge edge;
    // common_conversions::convertToProto(tform_base_to_frame.value().transform, *edge.mutable_parent_tform_child());

    if (world_object_names.count(tf_frame) == 0) {
      request.mutable_mutation()->set_action(
          ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_ADD);
    } else {
      request.mutable_mutation()->set_action(
          ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_CHANGE);
    }
    world_object_client_interface_->mutateWorldObject(request);
  }
}

}  // namespace spot_ros2
