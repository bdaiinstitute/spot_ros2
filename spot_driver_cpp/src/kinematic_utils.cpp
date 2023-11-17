// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic_utils.hpp>

namespace spot_ros2::kinematic_utils {

InverseKinematicsRequest convert_inverse_kinematics_request_to_proto(
    const std::shared_ptr<GetInverseKinematicSolutions::Request>& ros_msg) {}

std::shared_ptr<GetInverseKinematicSolutions::Response> convert_proto_to_inverse_kinematics_response(
    const InverseKinematicsResponse& proto) {}

}  // namespace spot_ros2::kinematic_utils
