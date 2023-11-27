// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>

#include <spot_driver_cpp/conversions/common_conversions.hpp>

namespace spot_ros2::kinematic_conversions {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf,

void convert_bosdyn_msgs_inverse_kinematics_request_fixed_stance_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestFixedStance& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::FixedStance& proto) {
  if (ros_msg.fl_rt_scene_is_set) {
    common_conversions::convert_geometry_msgs_vector3_to_proto(ros_msg.fl_rt_scene, *proto.mutable_fl_rt_scene());
  }
  if (ros_msg.fr_rt_scene_is_set) {
    common_conversions::convert_geometry_msgs_vector3_to_proto(ros_msg.fr_rt_scene, *proto.mutable_fr_rt_scene());
  }
  if (ros_msg.hl_rt_scene_is_set) {
    common_conversions::convert_geometry_msgs_vector3_to_proto(ros_msg.hl_rt_scene, *proto.mutable_hl_rt_scene());
  }
  if (ros_msg.hr_rt_scene_is_set) {
    common_conversions::convert_geometry_msgs_vector3_to_proto(ros_msg.hr_rt_scene, *proto.mutable_hr_rt_scene());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_on_ground_plane_stance_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOnGroundPlaneStance& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::OnGroundPlaneStance& proto) {
  if (ros_msg.scene_tform_ground_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_ground,
                                                            *proto.mutable_scene_tform_ground());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_one_of_stance_specification_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.stance_specification_choice == ros_msg.STANCE_SPECIFICATION_FIXED_STANCE_SET) {
    convert_bosdyn_msgs_inverse_kinematics_request_fixed_stance_to_proto(ros_msg.fixed_stance,
                                                                         *proto.mutable_fixed_stance());
  }
  if (ros_msg.stance_specification_choice == ros_msg.STANCE_SPECIFICATION_ON_GROUND_PLANE_STANCE_SET) {
    convert_bosdyn_msgs_inverse_kinematics_request_on_ground_plane_stance_to_proto(
        ros_msg.on_ground_plane_stance, *proto.mutable_on_ground_plane_stance());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_wrist_mounted_tool_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestWristMountedTool& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::WristMountedTool& proto) {
  if (ros_msg.wrist_tform_tool_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.wrist_tform_tool,
                                                            *proto.mutable_wrist_tform_tool());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_body_mounted_tool_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestBodyMountedTool& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::BodyMountedTool& proto) {
  if (ros_msg.body_tform_tool_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.body_tform_tool, *proto.mutable_body_tform_tool());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.tool_specification_choice == ros_msg.TOOL_SPECIFICATION_WRIST_MOUNTED_TOOL_SET) {
    convert_bosdyn_msgs_inverse_kinematics_request_wrist_mounted_tool_to_proto(ros_msg.wrist_mounted_tool,
                                                                               *proto.mutable_wrist_mounted_tool());
  }
  if (ros_msg.tool_specification_choice == ros_msg.TOOL_SPECIFICATION_BODY_MOUNTED_TOOL_SET) {
    convert_bosdyn_msgs_inverse_kinematics_request_body_mounted_tool_to_proto(ros_msg.body_mounted_tool,
                                                                              *proto.mutable_body_mounted_tool());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_tool_pose_task_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestToolPoseTask& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::ToolPoseTask& proto) {
  if (ros_msg.task_tform_desired_tool_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.task_tform_desired_tool,
                                                            *proto.mutable_task_tform_desired_tool());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_tool_gaze_task_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestToolGazeTask& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::ToolGazeTask& proto) {
  if (ros_msg.target_in_task_is_set) {
    common_conversions::convert_geometry_msgs_vector3_to_proto(ros_msg.target_in_task, *proto.mutable_target_in_task());
  }
  if (ros_msg.task_tform_desired_tool_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.task_tform_desired_tool,
                                                            *proto.mutable_task_tform_desired_tool());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.task_specification_choice == ros_msg.TASK_SPECIFICATION_TOOL_POSE_TASK_SET) {
    convert_bosdyn_msgs_inverse_kinematics_request_tool_pose_task_to_proto(ros_msg.tool_pose_task,
                                                                           *proto.mutable_tool_pose_task());
  }
  if (ros_msg.task_specification_choice == ros_msg.TASK_SPECIFICATION_TOOL_GAZE_TASK_SET) {
    convert_bosdyn_msgs_inverse_kinematics_request_tool_gaze_task_to_proto(ros_msg.tool_gaze_task,
                                                                           *proto.mutable_tool_gaze_task());
  }
}

void convert_bosdyn_msgs_inverse_kinematics_request_to_proto(const bosdyn_msgs::msg::InverseKinematicsRequest& ros_msg,
                                                             bosdyn::api::spot::InverseKinematicsRequest& proto) {
  if (ros_msg.header_is_set) {
    common_conversions::convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, *proto.mutable_header());
  }
  proto.set_root_frame_name(ros_msg.root_frame_name);
  if (ros_msg.root_tform_scene_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.root_tform_scene,
                                                            *proto.mutable_root_tform_scene());
  }
  if (ros_msg.scene_tform_task_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_task,
                                                            *proto.mutable_scene_tform_task());
  }

  // proto.nominal_arm_configuration = ros_msg.nominal_arm_configuration.value

  if (ros_msg.nominal_arm_configuration_overrides_is_set) {
    common_conversions::convert_bosdyn_msgs_arm_joint_position_to_proto(
        ros_msg.nominal_arm_configuration_overrides, *proto.mutable_nominal_arm_configuration_overrides());
  }
  if (ros_msg.scene_tform_body_nominal_is_set) {
    common_conversions::convert_geometry_msgs_pose_to_proto(ros_msg.scene_tform_body_nominal,
                                                            *proto.mutable_scene_tform_body_nominal());
  }

  convert_bosdyn_msgs_inverse_kinematics_request_one_of_stance_specification_to_proto(ros_msg.stance_specification,
                                                                                      proto);
  convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(ros_msg.tool_specification, proto);
  convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(ros_msg.task_specification, proto);
}

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convert_proto_to_bosdyn_msgs_request_header(const bosdyn::api::RequestHeader& proto,
                                                 bosdyn_msgs::msg::RequestHeader& ros_msg) {
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.request_timestamp(), ros_msg.request_timestamp);
  ros_msg.request_timestamp_is_set = proto.has_request_timestamp();
  ros_msg.client_name = proto.client_name();
  ros_msg.disable_rpc_logging = proto.disable_rpc_logging();
}

void convert_proto_to_bosdyn_msgs_common_error(const bosdyn::api::CommonError& proto,
                                               bosdyn_msgs::msg::CommonError& ros_msg) {
  ros_msg.code.value = proto.code();
  ros_msg.message = proto.message();
}

void convert_proto_to_bosdyn_msgs_response_header(const bosdyn::api::ResponseHeader& proto,
                                                  bosdyn_msgs::msg::ResponseHeader& ros_msg) {
  convert_proto_to_bosdyn_msgs_request_header(proto.request_header(), ros_msg.request_header);
  ros_msg.request_header_is_set = proto.has_request_header();
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.request_received_timestamp(),
                                                               ros_msg.request_received_timestamp);
  ros_msg.request_received_timestamp_is_set = proto.has_request_received_timestamp();
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.response_timestamp(), ros_msg.response_timestamp);
  ros_msg.response_timestamp_is_set = proto.has_response_timestamp();
  convert_proto_to_bosdyn_msgs_common_error(proto.error(), ros_msg.error);
  ros_msg.error_is_set = proto.has_error();
}

void convert_proto_to_bosdyn_msgs_joint_state(const bosdyn::api::JointState& proto,
                                              bosdyn_msgs::msg::JointState& ros_msg) {
  ros_msg.name = proto.name();
  ros_msg.position = proto.position().value();
  ros_msg.position_is_set = proto.has_position();
  ros_msg.velocity = proto.velocity().value();
  ros_msg.velocity_is_set = proto.has_velocity();
  ros_msg.acceleration = proto.acceleration().value();
  ros_msg.acceleration_is_set = proto.has_acceleration();
  ros_msg.load = proto.load().value();
  ros_msg.load_is_set = proto.has_load();
}

void convert_proto_to_bosdyn_msgs_frame_tree_snapshot_parent_edge(
    const bosdyn::api::FrameTreeSnapshot::ParentEdge& proto, bosdyn_msgs::msg::FrameTreeSnapshotParentEdge& ros_msg) {
  ros_msg.parent_frame_name = proto.parent_frame_name();
  common_conversions::convert_proto_to_geometry_msgs_pose(proto.parent_tform_child(), ros_msg.parent_tform_child);
  ros_msg.parent_tform_child_is_set = proto.has_parent_tform_child();
}

void convert_proto_to_bosdyn_msgs_frame_tree_snapshot(const bosdyn::api::FrameTreeSnapshot& proto,
                                                      bosdyn_msgs::msg::FrameTreeSnapshot& ros_msg) {
  ros_msg.child_to_parent_edge_map.clear();
  for (const auto& item : proto.child_to_parent_edge_map()) {
    bosdyn_msgs::msg::KeyStringValueBosdynMsgsFrameTreeSnapshotParentEdge edge;
    edge.key = item.first;
    convert_proto_to_bosdyn_msgs_frame_tree_snapshot_parent_edge(item.second, edge.value);
    ros_msg.child_to_parent_edge_map.push_back(edge);
  }
}

void convert_proto_to_bosdyn_msgs_kinematic_state(const bosdyn::api::KinematicState& proto,
                                                  bosdyn_msgs::msg::KinematicState& ros_msg) {
  ros_msg.joint_states.clear();
  for (const auto& item : proto.joint_states()) {
    bosdyn_msgs::msg::JointState joint_state;
    convert_proto_to_bosdyn_msgs_joint_state(item, joint_state);
    ros_msg.joint_states.push_back(joint_state);
  }
  common_conversions::convert_proto_to_builtin_interfaces_time(proto.acquisition_timestamp(),
                                                               ros_msg.acquisition_timestamp);
  ros_msg.acquisition_timestamp_is_set = proto.has_acquisition_timestamp();
  convert_proto_to_bosdyn_msgs_frame_tree_snapshot(proto.transforms_snapshot(), ros_msg.transforms_snapshot);
  ros_msg.transforms_snapshot_is_set = proto.has_transforms_snapshot();
  common_conversions::convert_proto_to_geometry_msgs_twist(proto.velocity_of_body_in_vision(),
                                                           ros_msg.velocity_of_body_in_vision);
  ros_msg.velocity_of_body_in_vision_is_set = proto.has_velocity_of_body_in_vision();
  common_conversions::convert_proto_to_geometry_msgs_twist(proto.velocity_of_body_in_odom(),
                                                           ros_msg.velocity_of_body_in_odom);
  ros_msg.velocity_of_body_in_odom_is_set = proto.has_velocity_of_body_in_odom();
}

void convert_proto_to_bosdyn_msgs_inverse_kinematics_response(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                                                              bosdyn_msgs::msg::InverseKinematicsResponse& ros_msg) {
  convert_proto_to_bosdyn_msgs_response_header(proto.header(), ros_msg.header);
  ros_msg.header_is_set = proto.has_header();
  ros_msg.status.value = proto.status();
  convert_proto_to_bosdyn_msgs_kinematic_state(proto.robot_configuration(), ros_msg.robot_configuration);
  ros_msg.robot_configuration_is_set = proto.has_robot_configuration();
}

}  // namespace spot_ros2::kinematic_conversions
