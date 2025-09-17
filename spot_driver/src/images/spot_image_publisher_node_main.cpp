// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <spot_driver/api/spot_clock_sources.hpp>
#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <memory>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<spot_ros2::images::SpotImagePublisherNode>();

  // Set the Spot SDK clock source to use the ROS clock.
  spot_ros2::SetSpotSDKClockSource(node->get_clock());

  // Spins the node with the default single-threaded executor.
  rclcpp::spin(node->get_node_base_interface());

  return 0;
}
