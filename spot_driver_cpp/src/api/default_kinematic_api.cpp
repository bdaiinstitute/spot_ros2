// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_kinematic_api.hpp>

namespace spot_ros2 {
DefaultKinematicApi::DefaultKinematicApi(std::shared_ptr<Robot> robot) : robot_{robot} {}

tl::expected<void, std::string> DefaultKinematicApi::init() {
  const auto kinematic_client_result =
      robot_->robot().EnsureServiceClient<InverseKinematicsClient>(InverseKinematicsClient::GetDefaultServiceName());
  if (!kinematic_client_result.status) {
    return tl::make_unexpected("Failed to initialize the Spot SDK inverse kinematic client.");
  }
  kinematic_client_.reset(kinematic_client_result.response);
}

tl::expected<Result<InverseKinematicsResponse>, std::string> DefaultKinematicApi::get_solutions(
    InverseKinematicsRequest& request) {
  auto result = kinematic_client_->InverseKinematics(request);
  if (!result.status) {
    return tl::make_unexpected("Failed to get solutios: " + result.status.DebugString());
  }
  return result;
}

}  // namespace spot_ros2
