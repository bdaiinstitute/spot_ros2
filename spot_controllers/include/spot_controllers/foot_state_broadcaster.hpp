// File modified. Modifications Copyright (c) 2025 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "spot_msgs/msg/foot_state.hpp"
#include "spot_msgs/msg/foot_state_array.hpp"

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "spot_controllers/foot_state_broadcaster_parameters.hpp"
#include "spot_controllers/spot_controller_utils.hpp"

namespace spot_controllers {
/**
 * \brief Foot State Broadcaster for Spot's feet contacts.
 *
 * FootStateBroadcaster publishes state interfaces from ros2_control as ROS messages.
 * This broadcaster will broadcast the following state interfaces:
 * `foot_sensor/foot_state.front.left`
 * `foot_sensor/foot_state.front.right`
 * `foot_sensor/foot_state.back.left`
 * `foot_sensor/foot_state.back.right`
 * The interface name by default will be prefixed with the namespace of the controller.
 * For example, a controller launched in in the namespace "spot" will claim the interfaces
 * `spot/foot_sensor/foot_state.front.left`
 * `spot/foot_sensor/foot_state.front.right`
 * `spot/foot_sensor/foot_state.back.left`
 * `spot/foot_sensor/foot_state.back.right`
 * To avoid this prefixing, you can set the `use_namespace_as_prefix` parameter to `false`.
 *
 * \param use_namespace_as_prefix Boolean flag to indicate whether to prefix the sensor name
 * with the namespace of the node. Defaults to `true`.
 *
 * Publishes to:
 * - \b feet (spot_msgs::msg::FootState): Array of feet states in the order FL, FR, BL, BR
 */
class FootStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  FootStateBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 protected:
  // Optional parameters
  using Params = foot_state_broadcaster::Params;
  using ParamListener = foot_state_broadcaster::ParamListener;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::FootStateArray>> foot_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<spot_msgs::msg::FootStateArray>> realtime_foot_state_publisher_;
};

}  // namespace spot_controllers
