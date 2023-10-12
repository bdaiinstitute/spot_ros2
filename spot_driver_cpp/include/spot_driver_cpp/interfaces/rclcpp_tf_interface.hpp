// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver_cpp/interfaces/tf_interface_base.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

namespace spot_ros2
{
  /**
   * @brief Implements TfInterfaceBase to use the rclcpp TF system.
   */
class RclcppTfInterface : public TfInterfaceBase
{
public:
  RclcppTfInterface(const std::shared_ptr<rclcpp::Node>& node);

  tl::expected<void, std::string> publishStaticTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) override;

private:
  /** @brief Broadcaster for static transforms. */
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  /** @brief Tracks transforms which are currently being broadcast through the static transform broadcaster. */
  std::set<std::pair<std::string, std::string>> current_static_transforms_;
};
}
