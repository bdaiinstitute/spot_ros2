// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/api/kinematic_api.hpp>
#include <spot_driver/kinematic/kinematic_service.hpp>

#include <string>

namespace spot_ros2::test {

class MockKinematicApi : public KinematicApi {
 public:
  MOCK_METHOD((tl::expected<InverseKinematicsResponse, std::string>), getSolutions,
              (InverseKinematicsRequest & request), (override));
};

}  // namespace spot_ros2::test
