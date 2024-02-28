// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <set>
#include <spot_driver/interfaces/robot_model_interface_base.hpp>
#include <string>

namespace spot_ros2::test {
class MockRobotModelInterface : public RobotModelInterfaceBase {
 public:
  MOCK_METHOD((tl::expected<std::set<std::string>, std::string>), getFrameIds, (), (const, override));
};
}  // namespace spot_ros2::test
