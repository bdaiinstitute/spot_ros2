// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/kinematic_api.hpp>

#include <tl_expected/expected.hpp>

#include <string>

namespace spot_ros2 {

class DefaultKinematicApi : public KinematicApi {
 public:
  explicit DefaultKinematicApi(bosdyn::client::InverseKinematicsClient* kinematic_client);

  /**
   * @brief Return a solution to the given request.
   */
  [[nodiscard]] tl::expected<InverseKinematicsResponse, std::string> getSolutions(
      InverseKinematicsRequest& request) override;

 private:
  bosdyn::client::InverseKinematicsClient* kinematic_client_;
};
}  // namespace spot_ros2
