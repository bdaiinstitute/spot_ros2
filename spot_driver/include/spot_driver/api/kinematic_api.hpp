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
 public:
  // KinematicApi is move-only
  KinematicApi() = default;
  KinematicApi(KinematicApi&& other) = default;
  KinematicApi(const KinematicApi&) = delete;
  KinematicApi& operator=(KinematicApi&& other) = default;
  KinematicApi& operator=(const KinematicApi&) = delete;

  virtual ~KinematicApi() = default;

  /**
   * Return a solution to the given request.
   */
  virtual tl::expected<InverseKinematicsResponse, std::string> getSolutions(InverseKinematicsRequest& request) = 0;
};
}  // namespace spot_ros2
