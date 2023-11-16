// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic_utils.hpp>

namespace spot_ros2::kinematic_utils {

::bosdyn::api::spot::InverseKinematicsRequest to_proto(
    const std::shared_ptr<GetInverseKinematicSolutions::Request> request) {
  return {};
}

// RosResponse toRos(const ProtoResponse& proto_response) {
//   RosResponse response;
//   if (proto_response.has_header()) {
//   }

//   return RosResponse();
// }
}  // namespace spot_ros2::kinematic_utils
