// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <memory>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<spot_ros2::images::SpotImagePublisherNode>();
  // This node uses a multithreaded executor because the timer to poll the images takes far too long to be able to reach
  // the desired framerate and blocks itself from reaching 15Hz
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  return 0;
}
