// Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <chrono>
#include <memory>

#include <bosdyn/common/time.h>

#include <rclcpp/clock.hpp>

namespace spot_ros2 {

void SetSpotSDKClockSource(rclcpp::Clock::SharedPtr clock) {
  bosdyn::common::SetClock([clock_ref = std::weak_ptr<rclcpp::Clock>(clock)]() {
    if (auto clock = clock_ref.lock()) {
      return std::chrono::nanoseconds(clock->now().nanoseconds());
    } else {
      bosdyn::common::RestoreDefaultClock();
      return bosdyn::common::NsecSinceEpoch();
    }
  });
}

}  // namespace spot_ros2
