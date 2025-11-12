// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/api/spot_clock_sources.hpp>
#include <spot_driver/image_stitcher/image_stitcher_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  spot_ros2::ImageStitcherNode node{rclcpp::NodeOptions()};
  spot_ros2::SetSpotSDKClockSource(node.get_clock());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());
  executor.spin();
  return 0;
}
