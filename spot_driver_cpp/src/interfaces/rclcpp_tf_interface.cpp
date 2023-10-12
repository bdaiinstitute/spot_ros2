// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/interfaces/rclcpp_tf_interface.hpp>

namespace spot_ros2
{
  RclcppTfInterface::RclcppTfInterface(const std::shared_ptr<rclcpp::Node>& node)
  : static_tf_broadcaster_{ node }
  {
  }

  tl::expected<void, std::string> RclcppTfInterface::publishStaticTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
  {
    // Keep track of which transforms the static transform publisher is already publishing.
    std::vector<geometry_msgs::msg::TransformStamped> new_transforms;
    for (const auto& transform : transforms)
    {
      const auto frame_id_pair = std::make_pair(transform.header.frame_id, transform.child_frame_id);
      // Do not publish transforms which have already previously been published.
      if (current_static_transforms_.count(frame_id_pair) == 0)
      {
        new_transforms.push_back(transform);
        current_static_transforms_.insert(current_static_transforms_.end(), frame_id_pair);
      }
    }
    static_tf_broadcaster_.sendTransform(new_transforms);
    return {};
  }

}
