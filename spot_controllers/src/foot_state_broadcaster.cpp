// File modified. Modifications Copyright (c) 2025 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2021 ros2_control development team
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

#include "spot_controllers/foot_state_broadcaster.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"

namespace rclcpp_lifecycle {
class State;
}  // namespace rclcpp_lifecycle

namespace spot_controllers {

FootStateBroadcaster::FootStateBroadcaster() {}

controller_interface::CallbackReturn FootStateBroadcaster::on_init() {
  if (!get_node()) {
    fprintf(stderr, "Failed to aquire node handle!\n");
    return controller_interface::CallbackReturn::ERROR;
  }
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FootStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FootStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const bool use_namespace_as_prefix = params_.use_namespace_as_prefix;
  std::string sensor_prefix = "";
  if (use_namespace_as_prefix) {
    sensor_prefix = get_prefix_from_namespace(get_node()->get_namespace());
  }
  // Append the chosen prefix to the sensor name
  const std::string sensor_name = sensor_prefix + "foot_sensor";
  RCLCPP_DEBUG(get_node()->get_logger(), "sensor name: %s", sensor_name.c_str());
  // export sensor interfaces. These are hardcoded, assuming the format from spot ros2 control tags
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.front.left");
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.front.right");
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.back.left");
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.back.right");
  return state_interfaces_config;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();

  try {
    foot_state_publisher_ =
        get_node()->create_publisher<spot_msgs::msg::FootStateArray>("~/feet", rclcpp::SystemDefaultsQoS());

    realtime_foot_state_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<spot_msgs::msg::FootStateArray>>(foot_state_publisher_);
  } catch (const std::exception& e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // initialize foot state message
  auto& feet_state_msg = realtime_foot_state_publisher_->msg_;
  feet_state_msg.states.clear();
  // update foot state message
  for (size_t i = 0; i < 4; ++i) {
    spot_msgs::msg::FootState foot_state;
    foot_state.contact = spot_msgs::msg::FootState::CONTACT_UNKNOWN;
    feet_state_msg.states.push_back(foot_state);
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FootStateBroadcaster::update(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (realtime_foot_state_publisher_ && realtime_foot_state_publisher_->trylock()) {
    auto& feet_state_msg = realtime_foot_state_publisher_->msg_;
    // update foot state message
    for (size_t i = 0; i < 4; ++i) {
      // this follows the same order as the state_interface_configuration
      // 0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
      const auto& state_interface = state_interfaces_.at(i);
      const std::string interface_name = state_interface.get_interface_name();
      const auto interface_value = state_interface.get_value();
      uint8_t contact = std::isnan(interface_value) ? spot_msgs::msg::FootState::CONTACT_UNKNOWN : interface_value;
      feet_state_msg.states.at(i).contact = contact;
    }
    realtime_foot_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::FootStateBroadcaster, controller_interface::ControllerInterface)
