// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/interfaces/timer_interface_base.hpp>

#include <chrono>
#include <functional>

namespace spot_ros2::test {
class MockTimerInterface : public TimerInterfaceBase {
 public:
  MOCK_METHOD(void, setTimer, (const std::chrono::duration<double>& period, const std::function<void()>& callback),
              (override));
  MOCK_METHOD(void, clearTimer, (), (override));

  void onSetTimer(const std::function<void()>& callback) { m_callback = callback; }
  void trigger() { m_callback(); }

  std::function<void()> m_callback;
};
}  // namespace spot_ros2::test
