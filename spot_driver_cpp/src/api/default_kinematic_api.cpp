// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_kinematic_api.hpp>

namespace spot_ros2 {
DefaultKinematicApi::DefaultKinematicApi(bosdyn::client::InverseKinematicsClient* kinematic_client)
    : kinematic_client_{kinematic_client} {}

tl::expected<Result<InverseKinematicsResponse>, std::string> DefaultKinematicApi::get_solutions(
    InverseKinematicsRequest& request) {
  auto result = kinematic_client_->InverseKinematics(request);
  if (!result.status) {
    return tl::make_unexpected("Failed to get solutios: " + result.status.DebugString());
  }
  return result;
}

}  // namespace spot_ros2
