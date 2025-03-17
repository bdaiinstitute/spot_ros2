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

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle {
class State;
}  // namespace rclcpp_lifecycle

namespace spot_controllers {
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

FootStateBroadcaster::FootStateBroadcaster() {}

controller_interface::CallbackReturn FootStateBroadcaster::on_init() {
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

  // slight hack to avoid more config parsing -- get the namespace from the node to use as the sensor prefix.
  const std::string node_namespace = get_node()->get_namespace();
  // namespace is "/" if there is none, else it's "/<namespace>". Erase the first char ("/") for easier parsing.
  const std::string trimmed_namespace = node_namespace.substr(1);
  // Now trimmed_namespace is either the empty string or "<namespace>"
  const std::string sensor_prefix = trimmed_namespace.empty() ? "" : trimmed_namespace + "/";
  // Append this prefix to the sensor name
  const std::string sensor_name = sensor_prefix + "foot_sensor";
  RCLCPP_WARN(get_node()->get_logger(), "sensor name: %s", sensor_name.c_str());
  // export state interfaces based on the prefix. They are hardcoded for the spot case.
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.back.left");
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.back.right");
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.front.left");
  state_interfaces_config.names.push_back(sensor_name + "/" + "foot_state.front.right");
  return state_interfaces_config;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();

  try {
    foot_state_publisher_ =
        get_node()->create_publisher<spot_msgs::msg::FootStateArray>("~/feet_states", rclcpp::SystemDefaultsQoS());

    realtime_foot_state_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<spot_msgs::msg::FootStateArray>>(foot_state_publisher_);
  } catch (const std::exception& e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // initialize foot state message
  const size_t num_feet = 4;
  // default initialization for foot state message
  auto& feet_state_msg = realtime_foot_state_publisher_->msg_;
  feet_state_msg.states.clear();
  // update joint state message and dynamic joint state message
  for (size_t i = 0; i < num_feet; ++i) {
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

controller_interface::return_type FootStateBroadcaster::update(const rclcpp::Time& time,
                                                               const rclcpp::Duration& /*period*/) {
  // for (const auto& state_interface : state_interfaces_) {
  //   std::string interface_name = state_interface.get_interface_name();
  //   RCLCPP_WARN(get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
  //   state_interface.get_value());
  // }

  // todo map the state interface above to the contact field below.
  // important to check std::isnan() on the value, and pass in unknown if it is.

  if (realtime_foot_state_publisher_ && realtime_foot_state_publisher_->trylock()) {
    auto& feet_state_msg = realtime_foot_state_publisher_->msg_;

    // update joint state message and dynamic joint state message
    for (size_t i = 0; i < 4; ++i) {
      feet_state_msg.states[i].contact = spot_msgs::msg::FootState::CONTACT_MADE;
    }
    realtime_foot_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::FootStateBroadcaster, controller_interface::ControllerInterface)
