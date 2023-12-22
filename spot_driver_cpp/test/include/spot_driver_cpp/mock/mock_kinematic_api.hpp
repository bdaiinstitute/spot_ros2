// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/api/kinematic_api.hpp>
#include <spot_driver_cpp/kinematic/kinematic_service.hpp>

#include <string>

namespace spot_ros2::test {

class MockKinematicApi : public KinematicApi {
  MOCK_METHOD((tl::expected<Result<InverseKinematicsResponse>, std::string>), get_solutions,
              (InverseKinematicsRequest & request), (override));
};

}  // namespace spot_ros2::test
