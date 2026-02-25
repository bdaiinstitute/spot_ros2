// Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <spot_driver/image_stitcher/image_stitcher.hpp>

namespace spot_ros2 {
class ImageStitcherNode {
 public:
  explicit ImageStitcherNode(const rclcpp::NodeOptions& options);

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

  /**
   * @brief Returns the Clock of this class's node.
   * @details This function exists to allow for the Spot SDK clock source to be derived from a node's clock.
   *
   * @return A shared_ptr to the Clock of the node.
   */
  std::shared_ptr<rclcpp::Clock> get_clock();

 private:
  void initialize(std::unique_ptr<CameraSynchronizerBase> synchronizer,
                  std::unique_ptr<TfListenerInterfaceBase> tf_listener, std::unique_ptr<LoggerInterfaceBase> logger,
                  std::unique_ptr<ParameterInterfaceBase> parameter_interface);

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<ImageStitcher> stitcher_;
};
}  // namespace spot_ros2
