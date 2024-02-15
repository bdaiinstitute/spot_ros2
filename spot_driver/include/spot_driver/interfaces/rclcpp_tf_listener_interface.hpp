// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <spot_driver/interfaces/tf_listener_interface_base.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Implements TfInterfaceBase to use the rclcpp TF system.
 */
class RclcppTfListenerInterface : public TfListenerInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppTfInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppTfInterface shares ownership of the shared_ptr.
   */
  explicit RclcppTfListenerInterface(const std::shared_ptr<rclcpp::Node>& node);

  std::vector<std::string> getAllFrameNames() override;

  tl::expected<geometry_msgs::msg::TransformStamped, std::string> lookupTransform(
      const std::string& parent, const std::string& child, const rclcpp::Time& timepoint,
      const rclcpp::Duration& timeout) override;

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};
}  // namespace spot_ros2
