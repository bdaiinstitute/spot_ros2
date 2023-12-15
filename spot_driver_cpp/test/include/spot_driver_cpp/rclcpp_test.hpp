// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace spot_ros2::test {
/**
 * @brief Test fixture that calls rclcpp::init() if needed.
 */
class RclcppTest : public ::testing::Test {
 public:
  /**
   * @brief Constructor for RclcppTest.
   * @details When the test fixture is created, call rclcpp::ok() to determine if the rclcpp context is already
   * initialized.
   */
  RclcppTest();

  /**
   * @brief If the rclcpp context was not already initialized when the test fixture was created, calls rclcpp::init().
   */
  void SetUp() override;

  /**
   * @brief If the rclcpp context was not already initialized when the test fixture was created, call
   * rclcpp::shutdown().
   */
  void TearDown() override;

 private:
  /**  @brief Set to true if rclcpp::ok() returns true when the test fixture is created. */
  const bool is_rclcpp_initialized_;
};
}  // namespace spot_ros2::test
