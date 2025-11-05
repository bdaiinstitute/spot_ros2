// Copyright (c) 2023 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <spot_driver/interfaces/clock_interface_base.hpp>

namespace spot_ros2::test {
class MockClockInterface : public ClockInterfaceBase {
 public:
  MOCK_METHOD(std::shared_ptr<rclcpp::Clock>, getClock, (), (override));
  MOCK_METHOD(rclcpp::Time, now, (), (override));
};
}  // namespace spot_ros2::test
