// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <spot_driver/interfaces/tf_listener_interface_base.hpp>
#include <string>
#include <vector>

namespace spot_ros2::test {
class MockTfListenerInterface : public TfListenerInterfaceBase {
 public:
  MOCK_METHOD((std::vector<std::string>), getAllFrameNames, (), (const, override));

  MOCK_METHOD((tl::expected<geometry_msgs::msg::TransformStamped, std::string>), lookupTransform,
              (const std::string& parent, const std::string& child, const rclcpp::Time& timepoint), (const, override));
};
}  // namespace spot_ros2::test
