// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_kinematic_api.hpp>

namespace spot_ros2 {
DefaultKinematicApi::DefaultKinematicApi(bosdyn::client::InverseKinematicsClient* kinematic_client)
    : kinematic_client_{kinematic_client} {}

tl::expected<InverseKinematicsResponse, std::string> DefaultKinematicApi::getSolutions(
    InverseKinematicsRequest& request) {
  try {
    auto result = kinematic_client_->InverseKinematics(request);
    if (result) {
      return result.response;
    }
    return tl::make_unexpected(result.status.message());
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to query the InverseKinematics service: " + std::string{ex.what()});
  }
}

}  // namespace spot_ros2
