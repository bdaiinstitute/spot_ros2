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

  kinematic_client_.reset(std::move(kinematic_client_result.response));
}

tl::expected<Result<InverseKinematicsResponse>, std::string> get_solution(InverseKinematicsRequest& requestrequest) {
  // TODO: convert the ROS request into a Protobuffer request.
  // TODO: send the protobuf request.
  // TODO: convert the Protobuf response into a ROS response.

  return {};
}

}  // namespace spot_ros2
