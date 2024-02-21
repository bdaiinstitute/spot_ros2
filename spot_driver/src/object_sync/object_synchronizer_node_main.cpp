// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <spot_driver/object_sync/object_synchronizer_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  spot_ros2::ObjectSynchronizerNode node;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());
  executor.spin();

  return 0;
}
