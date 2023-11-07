// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/inverse_kinematic_node.hpp>

namespace spot_ros2 {

InverseKinematicNode::InverseKinematicNode(const rclcpp::NodeOptions& node_options)
    : node_{std::make_shared<rclcpp::Node>("inverse_kinematic_node", node_options)} {}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> InverseKinematicNode::get_node_base_interface() {}

}  // namespace spot_ros2