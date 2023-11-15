// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/kinematic_api.hpp>

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>

#include <memory>

namespace spot_ros2 {
class KinematicService {
 public:
  explicit KinematicService(std::unique_ptr<KinematicApi> kinematic_api, std::shared_ptr<rclcpp::Node> node);

 private:
  std::unique_ptr<KinematicApi> kinematic_api_;
};
}  // namespace spot_ros2