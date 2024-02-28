// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <cstddef>
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

namespace {
constexpr auto kTfSyncPeriod = std::chrono::duration<double>{1.0 / 1.0};  // 1 Hz

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

std::string getAfterFirstSeparator(const std::string& input, const char separator) {
  const std::size_t index = input.find(separator);
  if (index == std::string::npos) {
    return input;
  }
  return input.substr(index + 1);
}
}  // namespace

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

  // if (!state_client_interface_) {
  //   logger_interface_->logError("Robot state interface not initialized.");
  //   return;
  // }

  // const auto robot_state = state_client_interface_->getRobotState();
  // if(!robot_state)
  // {
  //   logger_interface_->logError("Failed to get robot state.");
  //   return;
  // }

  // Get all TF frame IDs
  const auto tf_frame_names = tf_listener_interface_->getAllFrameNames();
  std::cout << "TF frames:\n";
  for (const auto& frame : tf_frame_names) {
    std::cout << "  " << frame << "\n";
  }
  std::cout << std::endl;

  // Get all frame IDs which are a part of Spot (I think this is fixed once we set whether Spot has an arm. Can retrieve
  // from the URDF published by RobotStatePublisher.) Get all the frame IDs which are published by Spot but cannot be
  // modified by this node (for example, AR tag poses).
  // The URDF does not contain the frames for the cameras, which are published by Spot.
  // const auto robot_model_frames = robot_model_interface_->getFrameIds();
  // if (!robot_model_frames) {
  //   logger_interface_->logError("Failed to get Spot frame IDs from robot model: " + robot_model_frames.error());
  //   return;
  // }

  // std::cout << "URDF frames:\n";
  // for (const auto& frame : robot_model_frames.value()) {
  //   std::cout << "  " << frame << "\n";
  // }
  // std::cout << std::endl;

  ::bosdyn::api::ListWorldObjectRequest request_non_mutable_frames;
  request_non_mutable_frames.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_APRILTAG);
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto non_mutable_frames_response = world_object_client_interface_->listWorldObjects(request_non_mutable_frames);
  if (!non_mutable_frames_response) {
    logger_interface_->logError("Failed to list apriltag objects.");
    return;
  }

  ::bosdyn::api::ListWorldObjectRequest request_mutable_frames;
  request_mutable_frames.add_object_type(::bosdyn::api::WorldObjectType::WORLD_OBJECT_UNKNOWN);
  // TODO(jschornak-bdai): request should set earliest timestamp for objects
  const auto mutable_frames_response = world_object_client_interface_->listWorldObjects(request_mutable_frames);
  if (!mutable_frames_response) {
    logger_interface_->logError("Failed to list other objects.");
    return;
  }
  // Each WorldObject has its own transform snapshot
  // non_mutable_frames_response.value().world_objects().at(0)

  std::set<std::string> world_object_frames;
  for (const auto& world_object : non_mutable_frames_response.value().world_objects()) {
    if (!world_object.has_transforms_snapshot()) {
      continue;
    }
    for (const auto& world_object_subframe : world_object.transforms_snapshot().child_to_parent_edge_map()) {
      // Exclude frames that are part of Spot's body. The transform snapshot for the AprilTags for example contains the
      // vision, odom, body, and vision frames upstream of the fiducial frames
      if (kSpotInternalFrames.count(world_object_subframe.first) == 0) {
        world_object_frames.insert(world_object_subframe.first);
      }
    }
  }

  std::cout << "world object frames:\n";
  for (const auto& frame : world_object_frames) {
    std::cout << "  " << frame << "\n";
  }
  std::cout << std::endl;

  // If a TF frame does not originate from Spot:
  //   If the frame is already included in the world objects:
  //     Mutate world objects to update the transform of the frame's object
  //   Else (frame is not a world object):
  //     Mutate world objects to add the frame as a new object

  for (const auto& tf_frame : tf_frame_names) {
    // const auto tf_frame_no_prefix = getAfterFirstSeparator(tf_frame, '/');
    const auto tf_frame_no_prefix = stripPrefix(tf_frame, frame_prefix_);

    std::cout << tf_frame_no_prefix << std::endl;
    if (kSpotInternalFrames.count(tf_frame_no_prefix) > 0) {
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
    *object->mutable_name() = tf_frame_no_prefix;
    auto* edge_map = object->mutable_transforms_snapshot()->mutable_child_to_parent_edge_map();

    ::bosdyn::api::FrameTreeSnapshot_ParentEdge edge;
    // common_conversions::convertToProto(tform_base_to_frame.value().transform, *edge.mutable_parent_tform_child());

    if (world_object_frames.count(tf_frame_no_prefix) == 0) {
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
