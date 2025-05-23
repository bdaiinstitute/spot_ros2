// File modified. Modifications Copyright (c) 2025 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2024 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/pose_sensor.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "spot_controllers/spot_controller_utils.hpp"
#include "spot_controllers/spot_pose_broadcaster_parameters.hpp"

namespace spot_controllers {

class SpotPoseBroadcaster : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Logs a ROS pose message to console
  void log_pose(const geometry_msgs::msg::Pose& pose);

  using Params = spot_pose_broadcaster::Params;
  using ParamListener = spot_pose_broadcaster::ParamListener;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::string frame_prefix_;

  std::unique_ptr<semantic_components::PoseSensor> vision_pose_sensor_;
  std::unique_ptr<semantic_components::PoseSensor> odom_pose_sensor_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> vision_realtime_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> odom_realtime_publisher_;

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_tf_publisher_;
};

}  // namespace spot_controllers
