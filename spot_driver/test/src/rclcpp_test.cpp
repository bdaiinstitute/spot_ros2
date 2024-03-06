// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/rclcpp_test.hpp>

namespace spot_ros2::test {
RclcppTest::RclcppTest() : is_rclcpp_initialized_{rclcpp::ok()} {}

void RclcppTest::SetUp() {
  if (!is_rclcpp_initialized_) {
    rclcpp::init(0, nullptr);
  }
}

void RclcppTest::TearDown() {
  if (!is_rclcpp_initialized_) {
    rclcpp::shutdown();
  }
}
}  // namespace spot_ros2::test
