// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/node.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Implements TfBroadcasterInterfaceBase to use the rclcpp TF system.
 */
class RclcppTfBroadcasterInterface : public TfBroadcasterInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppTfBroadcasterInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppTfBroadcasterInterface shares ownership of the shared_ptr.
   */
  explicit RclcppTfBroadcasterInterface(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Add new transforms to the StaticTransformBroadcaster.
   * @param transforms Transforms to publish as static transforms.
   */
  void updateStaticTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) override;

  /**
   * @brief Broadcast TF messages for dynamic transforms.
   * @param transforms Vector of transforms to broadcast.
   */
  void sendDynamicTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) override;

 private:
  /**
   * @brief Broadcaster for static transforms.
   * @details The rclcpp StaticTransformBroadcaster publishes onto a latched topic using the transient_local QoS
   * option, where the most recent message published onto the topic will be republished to new subscribers when
   * they appear on the network. Because of this, it is not necessary to manually republish static transforms at
   * a rate.
   *
   * The StaticTransformBroadcaster internally stores the transforms it has previously published. When sendTransform()
   * is called with a transform that has a child frame that is not represented among the existing stored transforms,
   * it adds this transforms to the stored transform. This means that calling sendTransform() with a set of transforms
   * that all have child frames that are already represented in the stored transforms will not update the values of any
   * of the stored transforms and will only republish the prior values of all stored transforms.
   *
   * These characteristics mean that we should only call sendTransform() with never-before-published transforms, to
   * minimize unnecessary calls to publish onto the /tf_static topic.
   */
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  tf2_ros::TransformBroadcaster dynamic_tf_broadcaster_;

  /**
   * @brief Tracks transforms which are currently being broadcast through the static transform broadcaster.
   * @details This is used to check if any transforms passed into updateStaticTransforms() are being published for the
   * very first time.
   */
  std::set<std::string> current_static_child_frames_;
};
}  // namespace spot_ros2
