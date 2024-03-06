// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/conversions/kinematic_conversions.hpp>

#include <spot_driver/conversions/common_conversions.hpp>

namespace spot_ros2 {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestFixedStance& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::FixedStance& proto) {
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequestFixedStance::FL_RT_SCENE_FIELD_SET) {
    convertToProto(ros_msg.fl_rt_scene, *proto.mutable_fl_rt_scene());
  }
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequestFixedStance::FR_RT_SCENE_FIELD_SET) {
    convertToProto(ros_msg.fr_rt_scene, *proto.mutable_fr_rt_scene());
  }
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequestFixedStance::HL_RT_SCENE_FIELD_SET) {
    convertToProto(ros_msg.hl_rt_scene, *proto.mutable_hl_rt_scene());
  }
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequestFixedStance::HR_RT_SCENE_FIELD_SET) {
    convertToProto(ros_msg.hr_rt_scene, *proto.mutable_hr_rt_scene());
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOnGroundPlaneStance& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::OnGroundPlaneStance& proto) {
  if (ros_msg.has_field &
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOnGroundPlaneStance::SCENE_TFORM_GROUND_FIELD_SET) {
    convertToProto(ros_msg.scene_tform_ground, *proto.mutable_scene_tform_ground());
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  switch (ros_msg.which) {
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification::
        STANCE_SPECIFICATION_FIXED_STANCE_SET:
      convertToProto(ros_msg.fixed_stance, *proto.mutable_fixed_stance());
      break;
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification::
        STANCE_SPECIFICATION_ON_GROUND_PLANE_STANCE_SET:
      convertToProto(ros_msg.on_ground_plane_stance, *proto.mutable_on_ground_plane_stance());
      break;
    default:
      break;
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestWristMountedTool& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::WristMountedTool& proto) {
  if (ros_msg.has_field &
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestWristMountedTool::WRIST_TFORM_TOOL_FIELD_SET) {
    convertToProto(ros_msg.wrist_tform_tool, *proto.mutable_wrist_tform_tool());
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestBodyMountedTool& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::BodyMountedTool& proto) {
  if (ros_msg.has_field &
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestBodyMountedTool::BODY_TFORM_TOOL_FIELD_SET) {
    convertToProto(ros_msg.body_tform_tool, *proto.mutable_body_tform_tool());
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfToolSpecification& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  switch (ros_msg.which) {
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfToolSpecification::
        TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET:
      convertToProto(ros_msg.wrist_mounted_tool, *proto.mutable_wrist_mounted_tool());
      break;
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfToolSpecification::
        TOOL_SPECIFICATION_BODY_MOUNTED_TOOL_SET:
      convertToProto(ros_msg.body_mounted_tool, *proto.mutable_body_mounted_tool());
      break;
    default:
      break;
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolPoseTask& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::ToolPoseTask& proto) {
  if (ros_msg.has_field &
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolPoseTask::TASK_TFORM_DESIRED_TOOL_FIELD_SET) {
    convertToProto(ros_msg.task_tform_desired_tool, *proto.mutable_task_tform_desired_tool());
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolGazeTask& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::ToolGazeTask& proto) {
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolGazeTask::TARGET_IN_TASK_FIELD_SET) {
    convertToProto(ros_msg.target_in_task, *proto.mutable_target_in_task());
  }
  if (ros_msg.has_field &
      bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolGazeTask::TASK_TFORM_DESIRED_TOOL_FIELD_SET) {
    convertToProto(ros_msg.task_tform_desired_tool, *proto.mutable_task_tform_desired_tool());
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  switch (ros_msg.which) {
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification::
        TASK_SPECIFICATION_TOOL_POSE_TASK_SET:
      convertToProto(ros_msg.tool_pose_task, *proto.mutable_tool_pose_task());
      break;
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification::
        TASK_SPECIFICATION_TOOL_GAZE_TASK_SET:
      convertToProto(ros_msg.tool_gaze_task, *proto.mutable_tool_gaze_task());
      break;
    default:
      break;
  }
}

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequest& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequest::HEADER_FIELD_SET) {
    convertToProto(ros_msg.header, *proto.mutable_header());
  }
  proto.set_root_frame_name(ros_msg.root_frame_name);
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequest::ROOT_TFORM_SCENE_FIELD_SET) {
    convertToProto(ros_msg.root_tform_scene, *proto.mutable_root_tform_scene());
  }
  if (ros_msg.has_field & bosdyn_spot_api_msgs::msg::InverseKinematicsRequest::SCENE_TFORM_TASK_FIELD_SET) {
    convertToProto(ros_msg.scene_tform_task, *proto.mutable_scene_tform_task());
  }

  switch (ros_msg.nominal_arm_configuration.value) {
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_UNKNOWN:
      proto.set_nominal_arm_configuration(bosdyn::api::spot::InverseKinematicsRequest::ARM_CONFIG_UNKNOWN);
      break;
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_READY:
      proto.set_nominal_arm_configuration(bosdyn::api::spot::InverseKinematicsRequest::ARM_CONFIG_READY);
      break;
    case bosdyn_spot_api_msgs::msg::InverseKinematicsRequestNamedArmConfiguration::ARM_CONFIG_CURRENT:
      proto.set_nominal_arm_configuration(bosdyn::api::spot::InverseKinematicsRequest::ARM_CONFIG_CURRENT);
      break;
  }

  using InverseKinematicsRequest = bosdyn_spot_api_msgs::msg::InverseKinematicsRequest;
  if (ros_msg.has_field & InverseKinematicsRequest::NOMINAL_ARM_CONFIGURATION_OVERRIDES_FIELD_SET) {
    convertToProto(ros_msg.nominal_arm_configuration_overrides, *proto.mutable_nominal_arm_configuration_overrides());
  }
  if (ros_msg.has_field & InverseKinematicsRequest::SCENE_TFORM_BODY_NOMINAL_FIELD_SET) {
    convertToProto(ros_msg.scene_tform_body_nominal, *proto.mutable_scene_tform_body_nominal());
  }

  convertToProto(ros_msg.stance_specification, proto);
  convertToProto(ros_msg.tool_specification, proto);
  convertToProto(ros_msg.task_specification, proto);
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convertToRos(const bosdyn::api::JointState& proto, bosdyn_api_msgs::msg::JointState& ros_msg) {
  ros_msg.has_field = 0u;
  ros_msg.name = proto.name();
  if (proto.has_position()) {
    ros_msg.position.data = proto.position().value();
    ros_msg.has_field |= bosdyn_api_msgs::msg::JointState::POSITION_FIELD_SET;
  }
  if (proto.has_velocity()) {
    ros_msg.velocity.data = proto.velocity().value();
    ros_msg.has_field |= bosdyn_api_msgs::msg::JointState::VELOCITY_FIELD_SET;
  }
  if (proto.has_acceleration()) {
    ros_msg.acceleration.data = proto.acceleration().value();
    ros_msg.has_field |= bosdyn_api_msgs::msg::JointState::ACCELERATION_FIELD_SET;
  }
  if (proto.has_load()) {
    ros_msg.load.data = proto.load().value();
    ros_msg.has_field |= bosdyn_api_msgs::msg::JointState::LOAD_FIELD_SET;
  }
}

void convertToRos(const bosdyn::api::FrameTreeSnapshot::ParentEdge& proto,
                  bosdyn_api_msgs::msg::FrameTreeSnapshotParentEdge& ros_msg) {
  ros_msg.has_field = 0u;
  ros_msg.parent_frame_name = proto.parent_frame_name();
  if (proto.has_parent_tform_child()) {
    convertToRos(proto.parent_tform_child(), ros_msg.parent_tform_child);
    ros_msg.has_field |= bosdyn_api_msgs::msg::FrameTreeSnapshotParentEdge::PARENT_TFORM_CHILD_FIELD_SET;
  }
}

void convertToRos(const bosdyn::api::FrameTreeSnapshot& proto, bosdyn_api_msgs::msg::FrameTreeSnapshot& ros_msg) {
  ros_msg.child_to_parent_edge_map.clear();
  for (const auto& item : proto.child_to_parent_edge_map()) {
    bosdyn_api_msgs::msg::FrameTreeSnapshotChildToParentEdgeMapEntry edge;
    edge.key = item.first;
    convertToRos(item.second, edge.value);
    ros_msg.child_to_parent_edge_map.push_back(edge);
  }
}

void convertToRos(const bosdyn::api::KinematicState& proto, bosdyn_api_msgs::msg::KinematicState& ros_msg) {
  ros_msg.has_field = 0u;
  ros_msg.joint_states.clear();
  for (const auto& item : proto.joint_states()) {
    bosdyn_api_msgs::msg::JointState joint_state;
    convertToRos(item, joint_state);
    ros_msg.joint_states.push_back(joint_state);
  }
  if (proto.has_acquisition_timestamp()) {
    convertToRos(proto.acquisition_timestamp(), ros_msg.acquisition_timestamp);
    ros_msg.has_field |= bosdyn_api_msgs::msg::KinematicState::ACQUISITION_TIMESTAMP_FIELD_SET;
  }
  if (proto.has_transforms_snapshot()) {
    convertToRos(proto.transforms_snapshot(), ros_msg.transforms_snapshot);
    ros_msg.has_field |= bosdyn_api_msgs::msg::KinematicState::TRANSFORMS_SNAPSHOT_FIELD_SET;
  }
  if (proto.has_velocity_of_body_in_vision()) {
    convertToRos(proto.velocity_of_body_in_vision(), ros_msg.velocity_of_body_in_vision);
    ros_msg.has_field |= bosdyn_api_msgs::msg::KinematicState::VELOCITY_OF_BODY_IN_VISION_FIELD_SET;
  }
  if (proto.has_velocity_of_body_in_odom()) {
    convertToRos(proto.velocity_of_body_in_odom(), ros_msg.velocity_of_body_in_odom);
    ros_msg.has_field |= bosdyn_api_msgs::msg::KinematicState::VELOCITY_OF_BODY_IN_ODOM_FIELD_SET;
  }
}

void convertToRos(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                  bosdyn_spot_api_msgs::msg::InverseKinematicsResponse& ros_msg) {
  ros_msg.has_field = 0u;
  if (proto.has_header()) {
    convertToRos(proto.header(), ros_msg.header);
    ros_msg.has_field |= bosdyn_spot_api_msgs::msg::InverseKinematicsResponse::HEADER_FIELD_SET;
  }
  ros_msg.status.value = proto.status();
  if (proto.has_robot_configuration()) {
    convertToRos(proto.robot_configuration(), ros_msg.robot_configuration);
    ros_msg.has_field |= bosdyn_spot_api_msgs::msg::InverseKinematicsResponse::ROBOT_CONFIGURATION_FIELD_SET;
  }
}

}  // namespace spot_ros2
