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
  //   for (const auto& joint : params_.joints) {
  //     for (const auto& interface : params_.interfaces) {
  //       state_interfaces_config.names.push_back(joint + "/" + interface);
  //     }
  //   }
  return state_interfaces_config;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  RCLCPP_INFO(get_node()->get_logger(), "CONFIGURING");

  try {
    const std::string topic_name_prefix = "";

    foot_state_publisher_ = get_node()->create_publisher<spot_msgs::msg::FootStateArray>(
        topic_name_prefix + "feet_states", rclcpp::SystemDefaultsQoS());

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
  // update joint state message and dynamic joint state message
  for (size_t i = 0; i < num_feet; ++i) {
    spot_msgs::msg::FootState foot_state;
    foot_state.contact = spot_msgs::msg::FootState::CONTACT_UNKNOWN;
    feet_state_msg.states[i] = foot_state;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FootStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FootStateBroadcaster::update(const rclcpp::Time& time,
                                                               const rclcpp::Duration& /*period*/) {
  //   for (const auto& state_interface : state_interfaces_) {
  //     std::string interface_name = state_interface.get_interface_name();
  //     if (map_interface_to_joint_state_.count(interface_name) > 0) {
  //       interface_name = map_interface_to_joint_state_[interface_name];
  //     }
  //     name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] = state_interface.get_value();
  //     RCLCPP_DEBUG(get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
  //     state_interface.get_value());
  //   }

  if (realtime_foot_state_publisher_ && realtime_foot_state_publisher_->trylock()) {
    auto& feet_state_msg = realtime_foot_state_publisher_->msg_;

    // update joint state message and dynamic joint state message
    for (size_t i = 0; i < 4; ++i) {
      spot_msgs::msg::FootState foot_state;
      foot_state.contact = spot_msgs::msg::FootState::CONTACT_MADE;
      feet_state_msg.states[i] = foot_state;
    }
    realtime_foot_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::FootStateBroadcaster, controller_interface::ControllerInterface)
