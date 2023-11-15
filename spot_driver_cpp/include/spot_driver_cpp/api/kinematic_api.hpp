// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/inverse_kinematics/inverse_kinematics_client.h>
#include <bosdyn/client/service_client/result.h>

#include <tl_expected/expected.hpp>

#include <string>

namespace spot_ros2 {

using ::bosdyn::api::spot::InverseKinematicsRequest;
using ::bosdyn::api::spot::InverseKinematicsResponse;
using ::bosdyn::client::Result;

class KinematicApi {
  /**
   * Return a solution to the given request.
   */
  virtual tl::expected<Result<InverseKinematicsResponse>, std::string> get_solution(
      InverseKinematicsRequest& requestrequest) = 0;
};
}  // namespace spot_ros2