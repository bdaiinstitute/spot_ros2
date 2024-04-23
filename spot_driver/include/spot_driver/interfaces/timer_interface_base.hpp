// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <chrono>
#include <functional>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that calls a callback function at a specified rate.
 */
class TimerInterfaceBase {
 public:
  // TimerInterfaceBase is move-only
  TimerInterfaceBase() = default;
  TimerInterfaceBase(TimerInterfaceBase&& other) = default;
  TimerInterfaceBase(const TimerInterfaceBase&) = delete;
  TimerInterfaceBase& operator=(TimerInterfaceBase&& other) = default;
  TimerInterfaceBase& operator=(const TimerInterfaceBase&) = delete;

  virtual ~TimerInterfaceBase() = default;

  virtual void setTimer(const std::chrono::duration<double>& period, const std::function<void()>& callback) = 0;
  virtual void clearTimer() = 0;
};
}  // namespace spot_ros2
