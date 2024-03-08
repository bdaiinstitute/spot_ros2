// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/interfaces/clock_interface_base.hpp>

#include <chrono>
#include <functional>

namespace spot_ros2::test {
class MockClockInterface : public ClockInterfaceBase {
 public:
  MOCK_METHOD(rclcpp::Time, now, (), (override));
};
}  // namespace spot_ros2::test
