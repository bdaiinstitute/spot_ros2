// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <cstddef>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/duration.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/conversions/common_conversions.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/types.hpp>
#include <string>
#include <tl_expected/expected.hpp>
#include <utility>
#include "spot_driver/conversions/time.hpp"

namespace {
constexpr auto kTfSyncPeriod = std::chrono::duration<double>{1.0 / 1.0};  // 1 Hz

/**
 * @brief All frame names which are considered internal to Spot.
 * @details It is surprisingly challenging to automatically generate a comprehensive list of Spot's internal frames,
 * since the authorities that manage these frames are scattered across several Spot API clients. Since we do not expect
 * Spot's internal frames to change dynamically, we have manually defined this list of frames instead.
 */
inline const std::set<std::string> kSpotInternalFrames{
    // Links for moving robot joints
    "arm_link_el0",
    "arm_link_el1",
    "arm_link_fngr",
    "arm_link_hr0",
    "arm_link_hr1",
    "arm_link_sh0",
    "arm_link_sh1",
    "arm_link_wr0",
    "arm_link_wr1",
    "base_link",
    "body",
    "front_rail",
    "rear_rail",
    "hand",
    "gpe",
    "front_left_hip",
    "front_left_upper_leg",
    "front_left_lower_leg",
    "front_right_hip",
    "front_right_upper_leg",
    "front_right_lower_leg",
    "rear_left_hip",
    "rear_left_upper_leg",
    "rear_left_lower_leg",
    "rear_right_hip",
    "rear_right_upper_leg",
    "rear_right_lower_leg",

    // Base frames
    "vision",
    "flat_body",
    "odom",

    // Links for vision frames
    "back_fisheye",
    "back",
    "head",
    "frontleft_fisheye",
    "frontleft",
    "frontright_fisheye",
    "frontright",
    "left_fisheye",
    "left",
    "right_fisheye",
    "right",
    "hand_color_image_sensor",
    "hand_depth_sensor",

    // Mysterious forbidden frames which I expect will be deprecated in SDK version 4.0.0
    "arm0.link_wr1",
    "link_wr1",
};

std::string stripPrefix(const std::string& input, const std::string& prefix) {
  const std::size_t prefix_index = input.find(prefix);
  if (prefix_index == std::string::npos) {
    return input;
  }
  if (prefix_index > 0) {
    return input;
  }

  return input.substr(prefix.size());
}

enum class MutationOperation {
  ADD,
  CHANGE,
};

::bosdyn::api::MutateWorldObjectRequest createMutationRequest(
    const geometry_msgs::msg::TransformStamped& base_tform_child, const std::string& preferred_base_frame,
    const std::string& child_frame_id_no_prefix, const google::protobuf::Duration& clock_skew,
    const MutationOperation& operation) {
  ::bosdyn::api::MutateWorldObjectRequest request;
  *request.mutable_header()->mutable_request_timestamp() =
      spot_ros2::localTimeToRobotTime(base_tform_child.header.stamp, clock_skew);
  *request.mutable_mutation()->mutable_object()->mutable_acquisition_time() = request.header().request_timestamp();

  auto* object = request.mutable_mutation()->mutable_object();
  spot_ros2::convertToProto(base_tform_child.header.stamp, *object->mutable_acquisition_time());
  *object->mutable_name() = child_frame_id_no_prefix;

  ::bosdyn::api::FrameTreeSnapshot_ParentEdge edge;
  *edge.mutable_parent_frame_name() = preferred_base_frame;
  spot_ros2::convertToProto(base_tform_child.transform, *edge.mutable_parent_tform_child());

  auto* edge_map = object->mutable_transforms_snapshot()->mutable_child_to_parent_edge_map();
  edge_map->insert(google::protobuf::MapPair{child_frame_id_no_prefix, edge});
  // Add the root frame
  edge_map->insert(google::protobuf::MapPair{preferred_base_frame, ::bosdyn::api::FrameTreeSnapshot_ParentEdge{}});

  if (operation == MutationOperation::ADD) {
    request.mutable_mutation()->set_action(
        ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_ADD);
  } else {
    request.mutable_mutation()->set_action(
        ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_CHANGE);
  }

  return request;
}
}  // namespace

namespace spot_ros2 {

ObjectSynchronizer::ObjectSynchronizer(const std::shared_ptr<WorldObjectClientInterface>& world_object_client_interface,
                                       const std::shared_ptr<TimeSyncApi>& time_sync_api,
                                       std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                       std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                       std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                                       std::unique_ptr<TimerInterfaceBase> timer_interface)
    : world_object_client_interface_{world_object_client_interface},
      time_sync_interface_{time_sync_api},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      tf_listener_interface_{std::move(tf_listener_interface)},
      timer_interface_{std::move(timer_interface)} {
  const auto spot_name = parameter_interface_->getSpotName();
  frame_prefix_ = spot_name.empty() ? "" : spot_name + "/";

  preferred_base_frame_ = stripPrefix(parameter_interface_->getPreferredOdomFrame(), frame_prefix_);
  preferred_base_frame_with_prefix_ = preferred_base_frame_.find('/') == std::string::npos
                                          ? spot_name + "/" + preferred_base_frame_
                                          : preferred_base_frame_;

  timer_interface_->setTimer(kTfSyncPeriod, [this]() {
    // listWorldObjectsCallback();
    onTimer();
  });
}

void ObjectSynchronizer::onTimer() {
  if (!world_object_client_interface_) {
    logger_interface_->logError("World object interface not initialized.");
    return;
  }

  // Get a list of all world objects which are managed by Spot itself or through other Spot operator tools.
  // The TF tree may contain frames from these objects, and we must not attempt to modify these objects.
  ::bosdyn::api::ListWorldObjectRequest request_non_mutable_objects;
  request_non_mutable_objects.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_APRILTAG);
  request_non_mutable_objects.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_DOCK);
  request_non_mutable_objects.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_DRAWABLE);
  request_non_mutable_objects.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_IMAGE_COORDINATES);
  request_non_mutable_objects.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_STAIRCASE);
  request_non_mutable_objects.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_USER_NOGO);
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto non_mutable_objects_response =
      world_object_client_interface_->listWorldObjects(request_non_mutable_objects);
  if (!non_mutable_objects_response) {
    logger_interface_->logError("Failed to list vision objects.");
    return;
  }
  std::set<std::string> non_mutable_frames;
  for (const auto& object : non_mutable_objects_response.value().world_objects()) {
    if (!object.has_transforms_snapshot()) {
      continue;
    }
    for (const auto& subframe : object.transforms_snapshot().child_to_parent_edge_map()) {
      if (kSpotInternalFrames.count(subframe.first) == 0) {
        non_mutable_frames.insert(subframe.first);
      }
    }
  }

  // Get a list of all world objects with type UNKNOWN.
  // These objects are not managed by Spot, so we can mutate them.
  ::bosdyn::api::ListWorldObjectRequest request_mutable_frames;
  request_mutable_frames.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_UNKNOWN);
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto mutable_frames_response = world_object_client_interface_->listWorldObjects(request_mutable_frames);
  if (!mutable_frames_response) {
    logger_interface_->logError("Failed to list other objects.");
    return;
  }
  std::set<std::string> mutable_object_names;
  for (const auto& world_object : mutable_frames_response.value().world_objects()) {
    if (!world_object.has_transforms_snapshot()) {
      continue;
    }
    mutable_object_names.insert(world_object.name());
  }

  // If a TF frame does not originate from Spot:
  //   If the frame is already included in the world objects:
  //     Mutate world objects to update the transform of the frame's object
  //   Else (frame is not a world object):
  //     Mutate world objects to add the frame as a new object

  const auto clock_skew_result = time_sync_interface_->getClockSkew();
  if (!clock_skew_result) {
    logger_interface_->logError(std::string{"Failed to get latest clock skew: "}.append(clock_skew_result.error()));
    return;
  }

  // Get a list of all frame IDs in the TF tree
  for (const auto& child_frame_id : tf_listener_interface_->getAllFrameNames()) {
    const auto child_frame_id_no_prefix = stripPrefix(child_frame_id, frame_prefix_);

    // Skip frames which are internal to Spot or which are from objects which we cannot mutate
    if (kSpotInternalFrames.count(child_frame_id_no_prefix) > 0 ||
        non_mutable_frames.count(child_frame_id_no_prefix) > 0) {
      continue;
    }

    // Get the transform from the preferred base frame to the current TF frame
    const auto base_tform_child =
        tf_listener_interface_->lookupTransform(preferred_base_frame_with_prefix_, child_frame_id, rclcpp::Time{0, 0},
                                                rclcpp::Duration{std::chrono::nanoseconds{0}});
    if (!base_tform_child) {
      logger_interface_->logWarn(base_tform_child.error());
      continue;
    }

    // Create a request to add or modify a WorldObject.
    // The transform snapshot of this WorldObject will contain a single transfrom from the preferred base frame to the
    // TF frame. The name of the WorldObject will match the frame ID of the TF frame. If there is already a WorldObject
    // whose name matches the current frame's frame ID, the request will modify this object. Otherwise, the request will
    // add a new object.
    const MutationOperation operation =
        mutable_object_names.count(child_frame_id_no_prefix) == 0 ? MutationOperation::ADD : MutationOperation::CHANGE;
    auto request = createMutationRequest(base_tform_child.value(), preferred_base_frame_, child_frame_id_no_prefix,
                                         clock_skew_result.value(), operation);
    // Send the request to the API's client interface to add the object in Spot's environment.
    const auto response = world_object_client_interface_->mutateWorldObject(request);
    if (!response) {
      logger_interface_->logWarn(std::string("Failed to modify world object: ").append(response.error()));
    }
  }
}

void ObjectSynchronizer::listWorldObjectsCallback() {
  ::bosdyn::api::ListWorldObjectRequest request;
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_APRILTAG);
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_DOCK);
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_DRAWABLE);
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_IMAGE_COORDINATES);
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_STAIRCASE);
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_USER_NOGO);
  request.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_UNKNOWN);
  const auto response = world_object_client_interface_->listWorldObjects(request);
  if (!response) {
    logger_interface_->logError("Failed to list objects.");
    return;
  }
  for (const auto& object : response->world_objects()) {
    logger_interface_->logInfo("There is a world object named " + object.name());
  }
}
}  // namespace spot_ros2
