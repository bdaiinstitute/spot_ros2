// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/interfaces/tf_interface_base.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>

namespace spot_ros2::test {
class MockTfInterface : public TfInterfaceBase {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), updateStaticTransforms,
              (const std::vector<geometry_msgs::msg::TransformStamped>& transforms), (override));
  MOCK_METHOD((tl::expected<void, std::string>), sendDynamicTransforms,
              (const std::vector<geometry_msgs::msg::TransformStamped>& transforms), (override));
};
}  // namespace spot_ros2::test
