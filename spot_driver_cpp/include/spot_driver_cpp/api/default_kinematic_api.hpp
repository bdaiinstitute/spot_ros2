// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/kinematic_api.hpp>
#include <spot_driver_cpp/api/robot.hpp>

namespace spot_ros2 {

using ::bosdyn::client::InverseKinematicsClient;

class DefaultKinematicApi : public KinematicApi {
 public:
  DefaultKinematicApi(std::shared_ptr<Robot> robot);

  /**
   * Initialize all Spot SDK clients.
   */
  tl::expected<void, std::string> init();

  /**
   * Return a solution to the given request.
   */
  tl::expected<Result<InverseKinematicsResponse>, std::string> get_solution(InverseKinematicsRequest& requestrequest);

 private:
  std::shared_ptr<Robot> robot_;
  std::unique_ptr<InverseKinematicsClient> kinematic_client_;
};
}  // namespace spot_ros2