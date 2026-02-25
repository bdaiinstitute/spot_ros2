// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <spot_driver/api/spot_clock_sources.hpp>
#include <spot_driver/object_sync/object_synchronizer_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  spot_ros2::ObjectSynchronizerNode node;

  // Set the Spot SDK clock source to use the ROS clock.
  spot_ros2::SetSpotSDKClockSource(node.get_clock());

  // This node uses a multithreaded executor because there are two separate timers and it is important that they do not
  // block each other.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());
  executor.spin();

  return 0;
}
