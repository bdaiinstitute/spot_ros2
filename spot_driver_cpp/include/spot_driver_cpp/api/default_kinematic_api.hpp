// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/kinematic_api.hpp>

#include <tl_expected/expected.hpp>

#include <string>

namespace spot_ros2 {

class DefaultKinematicApi : public KinematicApi {
 public:
  explicit DefaultKinematicApi(bosdyn::client::InverseKinematicsClient* kinematicClient);

  /**
   * Return a solution to the given request.
   */
  tl::expected<Result<InverseKinematicsResponse>, std::string> getSolutions(InverseKinematicsRequest& request);

 private:
  bosdyn::client::InverseKinematicsClient* kinematicClient_;
};
}  // namespace spot_ros2
