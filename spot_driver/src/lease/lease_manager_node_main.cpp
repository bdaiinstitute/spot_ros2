// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors.hpp>
#include <spot_driver/api/spot_clock_sources.hpp>
#include <spot_driver/lease/lease_manager_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  spot_ros2::lease::LeaseManagerNode node;

  // Set the Spot SDK clock source to use the ROS clock.
  spot_ros2::SetSpotSDKClockSource(node.get_clock());

  // Spins the node with the default single-threaded executor.
  rclcpp::spin(node.get_node_base_interface());

  return 0;
}
