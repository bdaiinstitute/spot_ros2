// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/clock.hpp>

namespace spot_ros2 {

/**
 * @brief Sets the Spot SDK clock source to use the provided ROS clock.
 * @details This function sets the Spot SDK clock source to use the provided ROS clock. This allows the Spot SDK to
 * synchronize its time with the ROS time, which is especially important when using simulated time in ROS. Note
 * that this setting applies process-wide for as along as the provided clock remains alive. Also note that setting
 * the given clock as custom clock source does not increase its reference count.
 *
 * @param clock A shared pointer to an rclcpp::Clock instance to use as the Spot SDK clock source.
 */
void SetSpotSDKClockSource(rclcpp::Clock::SharedPtr clock);

}  // namespace spot_ros2
