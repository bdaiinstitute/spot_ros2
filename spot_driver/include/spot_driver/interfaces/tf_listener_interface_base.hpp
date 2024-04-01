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
 * @brief Defines an interface for a class that can retrieve transform data.
 */
class TfListenerInterfaceBase {
 public:
  virtual ~TfListenerInterfaceBase() = default;

  /**
   * @brief Get all frame IDs known to the transform source.
   * @return A vector of all frame IDs known to the transform source.
   */
  [[nodiscard]] virtual std::vector<std::string> getAllFrameNames() const = 0;

  /**
   * @brief Look up a transform from a parent frame to a child frame at the specified timepoint.
   * @param parent Parent frame ID.
   * @param child Child frame ID.
   * @param timepoint Get a transform that is valid for this timestamp. Set an all-zero timepoint to get the latest
   * valid timestamp.
   * @param timeout Duration to wait for a valid transform to become available.
   * @return If successful, returns a transform following the convention parent_tform_child. If not successful, returns
   * an error message.
   */
  [[nodiscard]] virtual tl::expected<geometry_msgs::msg::TransformStamped, std::string> lookupTransform(
      const std::string& parent, const std::string& child, const rclcpp::Time& timepoint,
      const rclcpp::Duration& timeout) const = 0;
};
}  // namespace spot_ros2
