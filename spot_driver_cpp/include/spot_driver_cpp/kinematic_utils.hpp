// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/spot/inverse_kinematics.pb.h>

#include <bosdyn_msgs/msg/inverse_kinematics_request.hpp>
#include <bosdyn_msgs/msg/inverse_kinematics_response.hpp>

#include <spot_driver_cpp/kinematic_service.hpp>

namespace spot_ros2::kinematic_utils {

using ::bosdyn::api::spot::InverseKinematicsRequest;
using spot_msgs::srv::GetInverseKinematicSolutions;

InverseKinematicsRequest to_proto(const std::shared_ptr<GetInverseKinematicSolutions::Request> request);

// RosResponse toRos(const ProtoResponse& response);

}  // namespace spot_ros2::kinematic_utils