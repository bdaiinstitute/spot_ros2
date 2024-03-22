// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/object_sync/object_synchronizer.hpp>

#include <bosdyn/api/world_object.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <iterator>
#include <rclcpp/duration.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/conversions/common_conversions.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/conversions/time.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/types.hpp>
#include <string>
#include <tl_expected/expected.hpp>
#include <utility>

namespace {
constexpr auto kWorldObjectSyncPeriod = std::chrono::duration<double>{1.0};  // 1 Hz
constexpr auto kTfBroadcasterPeriod = std::chrono::duration<double>{0.1};    // 10 Hz

/**
 * @brief All frame names which are considered internal to Spot.
 * @details It is surprisingly challenging to automatically generate a comprehensive list of Spot's internal frames,
 * since the authorities that manage these frames are scattered across several Spot API clients. We do not expect
 * Spot's internal frames to change dynamically, so we have manually defined this list of frames instead.
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
    "link_wr1",
    "right_fisheye",
    "right",
    "hand_color_image_sensor",
    "hand_depth_sensor",

    // Mysterious forbidden frames which I expect will be deprecated in SDK version 4.0.0
    "arm0.link_wr1",
};

/**
 * @brief Given an input string and a prefix string which is a substring starting at the beginning of the input string,
 * return a new string which is the difference between the input string and the prefix string.
 * @param input
 * @param prefix
 * @return A new string which is the difference between the input string and the prefix string.
 */
std::string stripPrefix(const std::string& input, const std::string& prefix) {
  const std::size_t prefix_index = input.find(prefix);
  if (prefix_index == std::string::npos) {
    // The input does not contain the prefix
    return input;
  }
  if (prefix_index > 0) {
    // The input does contain the prefix substring, but it does not begin at the start of the input.
    // Return the unmodified input.
    return input;
  }

  return input.substr(prefix.size());
}

/**
 * @brief Create string messages corresponding to the values of the ValidateFrameTreeSnapshotStatus enum.
 * @param status Enum to convert.
 * @return The string corresponding to the enum.
 */
std::string toString(const ::bosdyn::api::ValidateFrameTreeSnapshotStatus& status) {
  using Status = ::bosdyn::api::ValidateFrameTreeSnapshotStatus;
  switch (status) {
    case Status::CYCLE:
      return "CYCLE";
    case Status::DISJOINT:
      return "DISJOINT";
    case Status::EMPTY:
      return "EMPTY";
    case Status::EMPTY_CHILD_FRAME_NAME:
      return "EMPTY_CHILD_FRAME_NAME";
    case Status::UNKNOWN_PARENT_FRAME_NAME:
      return "UNKNOWN_PARENT_FRAME_NAME";
    case Status::VALID:
      return "VALID";
    default:
      return "Unknown status code.";
  }
}

/**
 * @brief Create string messages corresponding to the values of the MutateWorldObjectResponse_Status enum.
 * @param status Enum to convert.
 * @return The string corresponding to the enum.
 */
std::string toString(const ::bosdyn::api::MutateWorldObjectResponse_Status& status) {
  using Status = ::bosdyn::api::MutateWorldObjectResponse_Status;
  switch (status) {
    case Status::MutateWorldObjectResponse_Status_STATUS_INVALID_MUTATION_ID:
      return "INVALID_MUTATION_ID";
    case Status::MutateWorldObjectResponse_Status_STATUS_INVALID_WORLD_OBJECT:
      return "INVALID_WORLD_OBJECT";
    case Status::MutateWorldObjectResponse_Status_STATUS_NO_PERMISSION:
      return "NO_PERMISSION";
    case Status::MutateWorldObjectResponse_Status_STATUS_OK:
      return "OK";
    case Status::MutateWorldObjectResponse_Status_STATUS_UNKNOWN:
      return "UNKNOWN";
    default:
      return "Unknown status code";
  }
}

/**
 * @brief Create a ListWorldObjectRequest that requests world objects of all the types that ObjectSynchronizer should
 * not modify.
 * @details While Spot's MutateWorldObject service will reject requests to modify objects that were added by other
 * entities, we filter out these non-mutable world objects beforehand to prevent sending mutation requests that are
 * destined to fail. The only WorldObject type which ObjectSynchronizer adds or modifies is the Drawable type.
 *
 * @return Request for info about WorldObjects which ObjectSynchronizer should not modify.
 */
::bosdyn::api::ListWorldObjectRequest createNonMutableObjectsRequest() {
  using Type = ::bosdyn::api::WorldObjectType;
  ::bosdyn::api::ListWorldObjectRequest request;
  request.add_object_type(Type::WORLD_OBJECT_APRILTAG);
  request.add_object_type(Type::WORLD_OBJECT_DOCK);
  request.add_object_type(Type::WORLD_OBJECT_IMAGE_COORDINATES);
  request.add_object_type(Type::WORLD_OBJECT_STAIRCASE);
  request.add_object_type(Type::WORLD_OBJECT_USER_NOGO);
  request.add_object_type(Type::WORLD_OBJECT_UNKNOWN);
  return request;
}

/**
 * @brief Create a ListWorldObjectRequest that requests world objects of all the types that ObjectSynchronizer
 * may modify.
 * @details Currently the only valid type of object is the Drawable object type.
 * @return Request for info about WorldObjects which ObjectSynchronizer may modify.
 */
::bosdyn::api::ListWorldObjectRequest createMutableObjectsRequest() {
  using Type = ::bosdyn::api::WorldObjectType;
  ::bosdyn::api::ListWorldObjectRequest request;
  request.add_object_type(Type::WORLD_OBJECT_DRAWABLE);
  return request;
}

/**
 * @brief Create a ListWorldObjectRequest that requests all types of world objects.
 * @return A request to list all WorldObject types.
 */
::bosdyn::api::ListWorldObjectRequest createAllObjectsRequest() {
  ::bosdyn::api::ListWorldObjectRequest request;
  request.mutable_object_type()->MergeFrom(createMutableObjectsRequest().object_type());
  request.mutable_object_type()->MergeFrom(createNonMutableObjectsRequest().object_type());
  return request;
}

/**
 * @brief Given a ListWorldObjectResponse message, create a list of all the frames listed in the transform snapshots
 * those objects.
 *
 * @param list_objects_response Input message to parse.
 * @return A set of all frame IDs which are listed in the transform snapshots of the input objects.
 */
std::set<std::string> getObjectFrames(const ::bosdyn::api::ListWorldObjectResponse& list_objects_response) {
  std::set<std::string> frames;
  for (const auto& object : list_objects_response.world_objects()) {
    if (!object.has_transforms_snapshot()) {
      continue;
    }
    for (const auto& subframe : object.transforms_snapshot().child_to_parent_edge_map()) {
      frames.insert(subframe.first);
    }
  }
  return frames;
}

/**
 * @brief Given a ListWorldObjectResponse message, create a map between the names and the IDs of these objects.
 *
 * @param list_objects_response Input message to parse.
 * @return A map where the keys are the names of the objects and the values are the IDs of the objects.
 */
std::map<std::string, int> getObjectNamesAndIDs(const ::bosdyn::api::ListWorldObjectResponse& list_objects_response) {
  std::map<std::string, int> names_to_ids;
  for (const auto& object : list_objects_response.world_objects()) {
    names_to_ids.insert(std::make_pair(object.name(), object.id()));
  }
  return names_to_ids;
}

::bosdyn::api::MutateWorldObjectRequest createBaseMutateObjectRequest(
    const geometry_msgs::msg::TransformStamped& base_tform_child, const std::string& preferred_base_frame,
    const std::string& child_frame_id_no_prefix, const google::protobuf::Duration& clock_skew) {
  ::bosdyn::api::MutateWorldObjectRequest request;
  *request.mutable_header()->mutable_request_timestamp() =
      spot_ros2::localTimeToRobotTime(base_tform_child.header.stamp, clock_skew);
  *request.mutable_mutation()->mutable_object()->mutable_acquisition_time() = request.header().request_timestamp();

  auto* object = request.mutable_mutation()->mutable_object();
  // object->mutable_acquisition_time()->CopyFrom(request.header().request_timestamp());
  object->mutable_object_lifetime()->set_seconds(5.0);

  *object->mutable_name() = child_frame_id_no_prefix;

  auto* properties = object->mutable_drawable_properties()->Add();
  properties->mutable_frame()->set_arrow_length(0.1);
  properties->mutable_frame()->set_arrow_radius(0.01);

  ::bosdyn::api::FrameTreeSnapshot_ParentEdge edge;
  *edge.mutable_parent_frame_name() = preferred_base_frame;
  spot_ros2::convertToProto(base_tform_child.transform, *edge.mutable_parent_tform_child());

  auto* edge_map = object->mutable_transforms_snapshot()->mutable_child_to_parent_edge_map();
  edge_map->insert(google::protobuf::MapPair{child_frame_id_no_prefix, edge});
  // Add the root frame
  edge_map->insert(google::protobuf::MapPair{preferred_base_frame, ::bosdyn::api::FrameTreeSnapshot_ParentEdge{}});

  return request;
}

/**
 * @brief Create a MutateWorldObjectRequest which adds a new WorldObject corresponding to a TF frame.
 *
 * @param base_tform_child
 * @param preferred_base_frame
 * @param child_frame_id_no_prefix
 * @param clock_skew
 * @return ::bosdyn::api::MutateWorldObjectRequest
 */
::bosdyn::api::MutateWorldObjectRequest createAddObjectRequest(
    const geometry_msgs::msg::TransformStamped& base_tform_child, const std::string& preferred_base_frame,
    const std::string& child_frame_id_no_prefix, const google::protobuf::Duration& clock_skew) {
  auto request =
      createBaseMutateObjectRequest(base_tform_child, preferred_base_frame, child_frame_id_no_prefix, clock_skew);
  request.mutable_mutation()->set_action(
      ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_ADD);
  return request;
}

/**
 * @brief Create a MutateWorldObjectRequest which modifies an existing WorldObject corresponding to a TF frame.
 *
 * @param base_tform_child
 * @param preferred_base_frame
 * @param child_frame_id_no_prefix
 * @param clock_skew
 * @param id
 * @return ::bosdyn::api::MutateWorldObjectRequest
 */
::bosdyn::api::MutateWorldObjectRequest createModifyObjectRequest(
    const geometry_msgs::msg::TransformStamped& base_tform_child, const std::string& preferred_base_frame,
    const std::string& child_frame_id_no_prefix, const google::protobuf::Duration& clock_skew, const int& id) {
  auto request =
      createBaseMutateObjectRequest(base_tform_child, preferred_base_frame, child_frame_id_no_prefix, clock_skew);
  request.mutable_mutation()->mutable_object()->set_id(id);
  request.mutable_mutation()->set_action(
      ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_CHANGE);
  return request;
}
}  // namespace

namespace spot_ros2 {

ObjectSynchronizer::ObjectSynchronizer(const std::shared_ptr<WorldObjectClientInterface>& world_object_client_interface,
                                       const std::shared_ptr<TimeSyncApi>& time_sync_api,
                                       std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                       std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                       std::unique_ptr<TfInterfaceBase> tf_broadcaster_interface,
                                       std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                                       std::unique_ptr<TimerInterfaceBase> world_object_update_timer,
                                       std::unique_ptr<TimerInterfaceBase> tf_broadcaster_timer,
                                       std::unique_ptr<ClockInterfaceBase> clock_interface)
    : world_object_client_interface_{world_object_client_interface},
      time_sync_interface_{time_sync_api},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      tf_broadcaster_interface_{std::move(tf_broadcaster_interface)},
      tf_listener_interface_{std::move(tf_listener_interface)},
      world_object_update_timer_{std::move(world_object_update_timer)},
      tf_broadcaster_timer_{std::move(tf_broadcaster_timer)},
      clock_interface_{std::move(clock_interface)} {
  const auto spot_name = parameter_interface_->getSpotName();
  frame_prefix_ = spot_name.empty() ? "" : spot_name + "/";

  preferred_base_frame_ = stripPrefix(parameter_interface_->getPreferredOdomFrame(), frame_prefix_);
  preferred_base_frame_with_prefix_ = preferred_base_frame_.find('/') == std::string::npos
                                          ? spot_name + "/" + preferred_base_frame_
                                          : preferred_base_frame_;

  world_object_update_timer_->setTimer(kWorldObjectSyncPeriod, [this]() {
    syncWorldObjects();
  });

  tf_broadcaster_timer_->setTimer(kTfBroadcasterPeriod, [this]() {
    broadcastWorldObjectTransforms();
  });
}

void ObjectSynchronizer::syncWorldObjects() {
  if (!world_object_client_interface_) {
    logger_interface_->logError("World object interface not initialized.");
    return;
  }

  // Get a list of all world objects which are managed by Spot itself or through other Spot operator tools.
  // The TF tree may contain frames from these objects, and we should not attempt to modify these objects. The Spot API
  // allows us to attempt to modify them but it will fail if we try
  ::bosdyn::api::ListWorldObjectRequest request_non_mutable_objects = createNonMutableObjectsRequest();
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto non_mutable_objects_response =
      world_object_client_interface_->listWorldObjects(request_non_mutable_objects);
  if (!non_mutable_objects_response) {
    logger_interface_->logError("Failed to list non-mutable objects: " + non_mutable_objects_response.error());
    return;
  }
  const auto non_mutable_frames_unfiltered = getObjectFrames(non_mutable_objects_response.value());
  std::set<std::string> non_mutable_frames;
  std::set_difference(non_mutable_frames_unfiltered.cbegin(), non_mutable_frames_unfiltered.cend(),
                      kSpotInternalFrames.cbegin(), kSpotInternalFrames.cend(),
                      std::inserter(non_mutable_frames, non_mutable_frames.end()));

  // Get a list of all world objects with type DRAWABLE. This is the only type of object that ObjectSynchronizer can add
  // or modify.
  ::bosdyn::api::ListWorldObjectRequest request_mutable_frames = createMutableObjectsRequest();
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto mutable_frames_response = world_object_client_interface_->listWorldObjects(request_mutable_frames);
  if (!mutable_frames_response) {
    logger_interface_->logError("Failed to list mutable objects: " + mutable_frames_response.error());
    return;
  }
  const auto mutable_objects = getObjectNamesAndIDs(mutable_frames_response.value());

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

    // Get the transform from the preferred base frame to the current TF frame.
    // We set the lookup timestamp to timepoint zero, which tells the TF buffer to give us the earliest-available
    // transform to this frame. We set the lookup timeout duration to zero, which causes the lookup to return a failure
    // case immediately if the transform is not available instead of blocking.
    const auto base_tform_child =
        tf_listener_interface_->lookupTransform(preferred_base_frame_with_prefix_, child_frame_id, rclcpp::Time{0, 0},
                                                rclcpp::Duration{std::chrono::nanoseconds{0}});
    if (!base_tform_child) {
      logger_interface_->logWarn(base_tform_child.error());
      continue;
    }

    // TODO(jschornak): Convert transform timestamps to host-relative time before performing this comparison.
    // As currently written this throws an exception.
    // const auto now = clock_interface_->now();
    // if (now - base_tform_child.value().header.stamp > rclcpp::Duration{10, 0}) {
    //   // Skip adding transforms that are stale
    //   logger_interface_->logInfo("transform is too old");
    //   continue;
    // }

    // Create a request to add or modify a WorldObject.
    // The transform snapshot of this WorldObject will contain a single transfrom from the preferred base frame to the
    // TF frame. The name of the WorldObject will match the frame ID of the TF frame. If there is already a WorldObject
    // whose name matches the current frame's frame ID, the request will modify this object. Otherwise, the request will
    // add a new object.
    ::bosdyn::api::MutateWorldObjectRequest request;
    if (mutable_objects.count(child_frame_id_no_prefix) > 0) {
      const auto id = mutable_objects.at(child_frame_id_no_prefix);
      request = createModifyObjectRequest(base_tform_child.value(), preferred_base_frame_, child_frame_id_no_prefix,
                                          clock_skew_result.value(), id);
      logger_interface_->logInfo("Modifying existing object for frame " + child_frame_id_no_prefix);
    } else {
      request = createAddObjectRequest(base_tform_child.value(), preferred_base_frame_, child_frame_id_no_prefix,
                                       clock_skew_result.value());
      logger_interface_->logInfo("Adding new object for frame " + child_frame_id_no_prefix);
    }

    // TODO(jschornak-bdai): delete this check in favor of unit test to validate request creator function
    if (const auto status = ::bosdyn::api::ValidateFrameTreeSnapshot(request.mutation().object().transforms_snapshot());
        status != ::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID) {
      logger_interface_->logWarn("Frame tree snapshot for object " + child_frame_id_no_prefix +
                                 " is not valid. Error code: " + toString(status));
      continue;
    }

    // Send the request to the API's client interface to add the object in Spot's environment.
    const auto response = world_object_client_interface_->mutateWorldObject(request);
    if (!response) {
      logger_interface_->logWarn(std::string("Failed to modify world object: ").append(response.error()));
      continue;
    }
    logger_interface_->logInfo(
        std::string("Response after modifying world object: ").append(toString(response->status())));

    if (response->status() != ::bosdyn::api::MutateWorldObjectResponse::STATUS_OK) {
      logger_interface_->logWarn(std::string("Failed to modify world object: ").append(toString(response->status())));
      continue;
    }

    // After successfully adding new WorldObject, add the frame ID for this object to the list of frames whose
    // corresponding world objects originate in this node.
    {
      std::lock_guard lock{managed_frames_mutex_};
      managed_frames_.insert(child_frame_id);
    }
  }
}

void ObjectSynchronizer::broadcastWorldObjectTransforms() {
  const auto clock_skew_result = time_sync_interface_->getClockSkew();
  if (!clock_skew_result) {
    logger_interface_->logError(std::string{"Failed to get latest clock skew: "}.append(clock_skew_result.error()));
    return;
  }

  auto request = createAllObjectsRequest();
  const auto response = world_object_client_interface_->listWorldObjects(request);
  if (!response) {
    logger_interface_->logError("Failed to list world objects: " + response.error());
    return;
  }

  const auto objects = getObjectNamesAndIDs(response.value());
  std::set<std::string, std::less<>> all_object_names;
  std::transform(objects.cbegin(), objects.cend(), std::inserter(all_object_names, all_object_names.end()),
                 [](const std::pair<std::string, int>& pair) {
                   return pair.first;
                 });

  // Create a set of names of objects which are not managed by this node
  std::set<std::string, std::less<>> non_managed_objects;
  {
    std::lock_guard lock{managed_frames_mutex_};
    std::set_difference(all_object_names.cbegin(), all_object_names.cend(), managed_frames_.cbegin(),
                        managed_frames_.cend(), std::inserter(non_managed_objects, non_managed_objects.end()));
  }

  for (const auto& object : response->world_objects()) {
    // Skip publishing TF for objects this node does not manage
    if (non_managed_objects.count(object.name()) == 0) {
      continue;
    }

    // Convert the object's frame tree snapshot into ROS TF frames
    const auto transforms = getTf(object.transforms_snapshot(), object.acquisition_time(), clock_skew_result.value(),
                                  frame_prefix_, preferred_base_frame_with_prefix_);
    if (!transforms) {
      logger_interface_->logWarn("Failed to get TF tree for object `" + object.name() + "`.");
      continue;
    }
    // Broadcast TF frames for this object
    tf_broadcaster_interface_->sendDynamicTransforms(transforms->transforms);
  }
}
}  // namespace spot_ros2
