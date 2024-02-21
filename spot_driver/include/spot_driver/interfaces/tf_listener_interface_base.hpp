// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Defines an interface for classes that consume transform data.
 */
class TfListenerInterfaceBase {
 public:
  virtual ~TfListenerInterfaceBase() = default;

  virtual std::vector<std::string> getAllFrameNames() = 0;

  virtual tl::expected<geometry_msgs::msg::TransformStamped, std::string> lookupTransform(
      const std::string& parent, const std::string& child, const rclcpp::Time& timepoint,
      const rclcpp::Duration& timeout) = 0;
};
}  // namespace spot_ros2
