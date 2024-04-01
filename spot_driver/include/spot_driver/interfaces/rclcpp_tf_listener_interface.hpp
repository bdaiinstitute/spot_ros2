// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

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
 * @brief Implements TfListenerInterfaceBase to use the rclcpp TF system.
 */
class RclcppTfListenerInterface : public TfListenerInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppTfListenerInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppTfListenerInterface shares ownership of the shared_ptr.
   */
  explicit RclcppTfListenerInterface(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Get the frame IDs of all frames known to the TF buffer.
   * @details This will return all frame IDs that have been received since the buffer was initialized.
   * @return A vector of frame IDs.
   */
  std::vector<std::string> getAllFrameNames() const override;

  /**
   * @brief Look up a transform from a parent frame to a child frame at the specified timepoint.
   * @param parent Parent frame ID.
   * @param child Child frame ID.
   * @param timepoint Get a transform that is valid for this timestamp. Setting an all-zero timepoint is equivalent to
   * passing tf2::timePointZero().
   * @param timeout Duration to wait for a valid transform to become available.
   * @return If successful, returns a transform following the convention parent_tform_child. If not successful, returns
   * an error message.
   */
  tl::expected<geometry_msgs::msg::TransformStamped, std::string> lookupTransform(
      const std::string& parent, const std::string& child, const rclcpp::Time& timepoint,
      const rclcpp::Duration& timeout) const override;

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};
}  // namespace spot_ros2
