// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <memory>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<spot_ros2::images::SpotImagePublisherNode>();

  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
