// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/spot/inverse_kinematics.pb.h>

#include <bosdyn_msgs/msg/inverse_kinematics_request.hpp>
#include <bosdyn_msgs/msg/inverse_kinematics_response.hpp>

namespace spot_ros2::kinematic_conversions {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convertBosdynMsgsInverseKinematicsRequestFixedStanceToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestFixedStance& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::FixedStance& proto);

void convertBosdynMsgsInverseKinematicsRequestOnGroundPlaneStanceToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOnGroundPlaneStance& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::OnGroundPlaneStance& proto);

void convertBosdynMsgsInverseKinematicsRequestOneOfStanceSpecificationToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convertBosdynMsgsInverseKinematicsRequestWristMountedToolToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestWristMountedTool& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::WristMountedTool& proto);

void convertBosdynMsgsInverseKinematicsRequestBodyMountedToolToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestBodyMountedTool& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::BodyMountedTool& proto);

void convertBosdynMsgsInverseKinematicsRequestOneOfToolSpecificationToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfToolSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convertBosdynMsgsInverseKinematicsRequestToolPoseTaskToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestToolPoseTask& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::ToolPoseTask& proto);

void convertBosdynMsgsInverseKinematicsRequestToolGazeTaskToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestToolGazeTask& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest::ToolGazeTask& proto);

void convertBosdynMsgsInverseKinematicsRequestOneOfTaskSpecificationToProto(
    const bosdyn_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification& ros_msg,
    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convertBosdynMsgsInverseKinematicsRequestToProto(const bosdyn_msgs::msg::InverseKinematicsRequest& ros_msg,
                                                      bosdyn::api::spot::InverseKinematicsRequest& proto);

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convertProtoToBosdynMsgsJointState(const bosdyn::api::JointState& proto, bosdyn_msgs::msg::JointState& ros_msg);

void convertProtoToBosdynMsgsFrameTreeSnapshotParentEdge(const bosdyn::api::FrameTreeSnapshot::ParentEdge& proto,
                                                         bosdyn_msgs::msg::FrameTreeSnapshotParentEdge& ros_msg);

void convertProtoToBosdynMsgsFrameTreeSnapshot(const bosdyn::api::FrameTreeSnapshot& proto,
                                               bosdyn_msgs::msg::FrameTreeSnapshot& ros_msg);

void convertProtoToBosdynMsgsKinematicState(const bosdyn::api::KinematicState& proto,
                                            bosdyn_msgs::msg::KinematicState& ros_msg);

void convertProtoToBosdynMsgsInverseKinematicsResponse(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                                                       bosdyn_msgs::msg::InverseKinematicsResponse& ros_msg);

}  // namespace spot_ros2::kinematic_conversions
