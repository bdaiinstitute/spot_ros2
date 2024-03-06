// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/spot/inverse_kinematics.pb.h>

#include <bosdyn_spot_api_msgs/msg/inverse_kinematics_request.hpp>
#include <bosdyn_spot_api_msgs/msg/inverse_kinematics_response.hpp>

namespace spot_ros2 {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestFixedStance& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::FixedStance& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOnGroundPlaneStance& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::OnGroundPlaneStance& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfStanceSpecification& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestWristMountedTool& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::WristMountedTool& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestBodyMountedTool& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::BodyMountedTool& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfToolSpecification& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolPoseTask& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::ToolPoseTask& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestToolGazeTask& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest::ToolGazeTask& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequestOneOfTaskSpecification& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto);

void convertToProto(const bosdyn_spot_api_msgs::msg::InverseKinematicsRequest& ros_msg,
                    bosdyn::api::spot::InverseKinematicsRequest& proto);

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convertToRos(const bosdyn::api::JointState& proto, bosdyn_api_msgs::msg::JointState& ros_msg);

void convertToRos(const bosdyn::api::FrameTreeSnapshot::ParentEdge& proto,
                  bosdyn_api_msgs::msg::FrameTreeSnapshotParentEdge& ros_msg);

void convertToRos(const bosdyn::api::FrameTreeSnapshot& proto, bosdyn_api_msgs::msg::FrameTreeSnapshot& ros_msg);

void convertToRos(const bosdyn::api::KinematicState& proto, bosdyn_api_msgs::msg::KinematicState& ros_msg);

void convertToRos(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                  bosdyn_spot_api_msgs::msg::InverseKinematicsResponse& ros_msg);

}  // namespace spot_ros2
