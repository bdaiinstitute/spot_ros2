// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/interfaces/robot_state_client_interface.hpp>

#include <string>

namespace spot_ros2::test {
class MockRobotStateClient : public RobotStateClientInterface {
 public:
  MOCK_METHOD((tl::expected<RobotState, std::string>), getRobotState, (const std::string& preferred_odom_frame), (override));
};
}  // namespace spot_ros2::test
