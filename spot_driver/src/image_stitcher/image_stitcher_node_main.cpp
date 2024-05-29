// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/image_stitcher/image_stitcher_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  spot_ros2::ImageStitcherNode node{rclcpp::NodeOptions()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());
  executor.spin();
  return 0;
}
