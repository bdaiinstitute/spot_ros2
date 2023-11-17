// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/spot/inverse_kinematics.pb.h>

#include <bosdyn_msgs/msg/inverse_kinematics_request.hpp>
#include <bosdyn_msgs/msg/inverse_kinematics_response.hpp>

#include <spot_driver_cpp/kinematic_service.hpp>

namespace spot_ros2::kinematic_utils {

using ::bosdyn::api::spot::InverseKinematicsRequest;
using ::bosdyn::api::spot::InverseKinematicsResponse;
using spot_msgs::srv::GetInverseKinematicSolutions;

InverseKinematicsRequest convert_inverse_kinematics_request_to_proto(
    const std::shared_ptr<GetInverseKinematicSolutions::Request>& ros_msg);

std::shared_ptr<GetInverseKinematicSolutions::Response> convert_proto_to_inverse_kinematics_response(
    const InverseKinematicsResponse& proto);

}  // namespace spot_ros2::kinematic_utils