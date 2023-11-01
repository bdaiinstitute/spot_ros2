// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_publisher_node.hpp>

namespace spot_ros2 {
SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options)
    : node_{std::make_shared<rclcpp::Node>("image_publisher", node_options)}, internal_{SpotImagePublisher{node_}} {
  internal_.initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}
}  // namespace spot_ros2
