// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Defines an interface for classes that broadcast transform data.
 */
class TfBroadcasterInterfaceBase {
 public:
  // TfBroadcasterInterfaceBase is move-only
  TfBroadcasterInterfaceBase() = default;
  TfBroadcasterInterfaceBase(TfBroadcasterInterfaceBase&& other) = default;
  TfBroadcasterInterfaceBase(const TfBroadcasterInterfaceBase&) = delete;
  TfBroadcasterInterfaceBase& operator=(TfBroadcasterInterfaceBase&& other) = default;
  TfBroadcasterInterfaceBase& operator=(const TfBroadcasterInterfaceBase&) = delete;

  virtual ~TfBroadcasterInterfaceBase() = default;

  virtual void updateStaticTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) = 0;

  virtual void sendDynamicTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) = 0;
};
}  // namespace spot_ros2
