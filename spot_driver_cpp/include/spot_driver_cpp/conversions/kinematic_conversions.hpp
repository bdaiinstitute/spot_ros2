// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/spot/inverse_kinematics.pb.h>

#include <bosdyn_msgs/msg/inverse_kinematics_request.hpp>
#include <bosdyn_msgs/msg/inverse_kinematics_response.hpp>

#include <spot_driver_cpp/kinematic_service.hpp>

namespace spot_ros2::kinematic_conversions {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convert_bosdyn_msgs_inverse_kinematics_request_fixed_stance_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestFixedStance& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::FixedStance& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_on_ground_plane_stance_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOnGroundPlaneStance& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::OnGroundPlaneStance& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_one_of_stance_specification_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_wrist_mounted_tool_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestWristMountedTool& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::WristMountedTool& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_body_mounted_tool_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestBodyMountedTool& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::BodyMountedTool& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_one_of_tool_specification_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_tool_pose_task_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestToolPoseTask& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::ToolPoseTask& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_tool_gaze_task_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestToolGazeTask& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::ToolGazeTask& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_one_of_task_specification_to_proto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convert_bosdyn_msgs_inverse_kinematics_request_to_proto(const bosdyn_msgs::msg::InverseKinematicsRequest& ros_msg,
                                                             bosdyn::api::spot::InverseKinematicsRequest& proto);

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convert_proto_to_bosdyn_msgs_request_header(const bosdyn::api::RequestHeader& proto,
                                                 bosdyn_msgs::msg::RequestHeader& ros_msg);

void convert_proto_to_bosdyn_msgs_common_error(const bosdyn::api::CommonError& proto,
                                               bosdyn_msgs::msg::CommonError& ros_msg);

void convert_proto_to_bosdyn_msgs_response_header(const bosdyn::api::ResponseHeader& proto,
                                                  bosdyn_msgs::msg::ResponseHeader& ros_msg);

void convert_proto_to_bosdyn_msgs_joint_state(const bosdyn::api::JointState& proto,
                                              bosdyn_msgs::msg::JointState& ros_msg);

void convert_proto_to_bosdyn_msgs_frame_tree_snapshot_parent_edge(
    const bosdyn::api::FrameTreeSnapshot::ParentEdge& proto, bosdyn_msgs::msg::FrameTreeSnapshotParentEdge& ros_msg);

void convert_proto_to_bosdyn_msgs_frame_tree_snapshot(const bosdyn::api::FrameTreeSnapshot& proto,
                                                      bosdyn_msgs::msg::FrameTreeSnapshot& ros_msg);

void convert_proto_to_bosdyn_msgs_kinematic_state(const bosdyn::api::KinematicState& proto,
                                                  bosdyn_msgs::msg::KinematicState& ros_msg);

void convert_proto_to_bosdyn_msgs_inverse_kinematics_response(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                                                              bosdyn_msgs::msg::InverseKinematicsResponse& ros_msg);

}  // namespace spot_ros2::kinematic_conversions