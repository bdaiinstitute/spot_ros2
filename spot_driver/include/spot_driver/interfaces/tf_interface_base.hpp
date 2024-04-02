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
class TfInterfaceBase {
 public:
  // TfInterfaceBase is move-only
  TfInterfaceBase() = default;
  TfInterfaceBase(TfInterfaceBase&& other) = default;
  TfInterfaceBase(const TfInterfaceBase&) = delete;
  TfInterfaceBase& operator=(TfInterfaceBase&& other) = default;
  TfInterfaceBase& operator=(const TfInterfaceBase&) = delete;

  virtual ~TfInterfaceBase() = default;

  virtual void updateStaticTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) = 0;

  virtual void sendDynamicTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) = 0;
};
}  // namespace spot_ros2
