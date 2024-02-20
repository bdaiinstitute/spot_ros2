// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/interfaces/publisher_interface_base.hpp>

#include <chrono>
#include <functional>

namespace spot_ros2::test {
template <typename T>
class MockPublisherInterface : public PublisherInterfaceBase<T> {
 public:
  MOCK_METHOD(void, publish, (const T& message), (const, override));
};
}  // namespace spot_ros2::test
