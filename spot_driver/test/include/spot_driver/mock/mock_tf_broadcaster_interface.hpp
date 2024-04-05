// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>
#include <vector>

namespace spot_ros2::test {
class MockTfBroadcasterInterface : public TfBroadcasterInterfaceBase {
 public:
  MOCK_METHOD(void, updateStaticTransforms, (const std::vector<geometry_msgs::msg::TransformStamped>& transforms),
              (override));
  MOCK_METHOD(void, sendDynamicTransforms, (const std::vector<geometry_msgs::msg::TransformStamped>& transforms),
              (override));
};
}  // namespace spot_ros2::test
