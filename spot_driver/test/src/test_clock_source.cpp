// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>

#include <bosdyn/common/time.h>

#include <spot_driver/api/spot_clock_sources.hpp>

namespace spot_ros2::test {

class SpotClockSourceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
  }
  rclcpp::Clock::SharedPtr clock_;
};

TEST_F(SpotClockSourceTest, SetAndRestoreSpotSDKClockSource) {
  // GIVEN a ROS clock with a specific time override
  const rcl_time_point_value_t override_time = 1234567890987654321;  // nanoseconds
  ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), override_time), RCL_RET_OK);
  // WHEN setting the Spot SDK clock source
  SetSpotSDKClockSource(clock_);
  // THEN bosdyn::common::NsecSinceEpoch should match the override
  EXPECT_EQ(bosdyn::common::NsecSinceEpoch().count(), override_time);

  // WHEN the clock is destroyed
  clock_.reset();
  // THEN bosdyn::common::NsecSinceEpoch should still return a value (restored default)
  EXPECT_GT(bosdyn::common::NsecSinceEpoch().count(), 0);
}

}  // namespace spot_ros2::test
