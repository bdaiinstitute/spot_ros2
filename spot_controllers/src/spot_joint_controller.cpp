// File modified. Modifications Copyright (c) 2025 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "spot_controllers/spot_joint_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace spot_controllers {
SpotJointController::SpotJointController()
    : controller_interface::ControllerInterface(), rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr) {}

controller_interface::CallbackReturn SpotJointController::on_init() {
  auto param_listener = std::make_shared<ParamListener>(get_node());
  params_ = param_listener->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotJointController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>("~/joint_commands", rclcpp::SystemDefaultsQoS(),
                                                                        [this](const CmdType::SharedPtr msg) {
                                                                          rt_command_ptr_.writeFromNonRT(msg);
                                                                        });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SpotJointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : params_.joints) {
    // these are the interfaces that get claimed
    command_interfaces_config.names.push_back(joint + "/" + "position");
    command_interfaces_config.names.push_back(joint + "/" + "velocity");
    command_interfaces_config.names.push_back(joint + "/" + "effort");
    command_interfaces_config.names.push_back(joint + "/" + "k_q_p");
    command_interfaces_config.names.push_back(joint + "/" + "k_qd_p");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SpotJointController::state_interface_configuration() const {
  // this controller has no knowledge of robot state... but it could (i.e. basic command validation?)
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn SpotJointController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, command_interface_types_, std::string(""),
                                                    ordered_interfaces) ||
      command_interface_types_.size() != ordered_interfaces.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu", command_interface_types_.size(),
                 ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotJointController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpotJointController::update(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands)) {
    return controller_interface::return_type::OK;
  }

  const auto& joint_names = (*joint_commands)->name;
  const auto& njoints_to_command = joint_names.size();

  const bool using_position = (*joint_commands)->position.size() == njoints_to_command;
  const bool using_velocity = (*joint_commands)->velocity.size() == njoints_to_command;
  const bool using_effort = (*joint_commands)->effort.size() == njoints_to_command;
  const bool using_k_q_p = (*joint_commands)->k_q_p.size() == njoints_to_command;
  const bool using_k_qd_p = (*joint_commands)->k_qd_p.size() == njoints_to_command;

  for (size_t i = 0; i < command_interfaces_.size(); i++) {
    // for every command interface, we wnat to tell if the received JointCommand contains an update for it.
    // First get the joint name (i.e. arm_sh0) and the interface name of this command interface (i.e. position)
    const auto& joint_name = command_interfaces_.at(i).get_prefix_name();
    const auto& interface_name = command_interfaces_.at(i).get_interface_name();
    // check if we want to command an interface on this joint by seeing if it's in the JointCommand message.
    const auto& it = std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (it == joint_names.end()) {
      // it wasn't in the JointCommand message, on to the next.
      continue;
    }
    // get the index that the name is at in the command
    const auto command_index = std::distance(joint_names.begin(), it);
    // check if interface name is there and update accordingly.
    // it's possible none of these are true (i.e. we are only using_position, but this specific interface is effort)
    // then we just keep going onto the next interface to check again.
    if (interface_name == "position" && using_position) {
      command_interfaces_.at(i).set_value((*joint_commands)->position.at(command_index));
    } else if (interface_name == "velocity" && using_velocity) {
      command_interfaces_.at(i).set_value((*joint_commands)->velocity.at(command_index));
    } else if (interface_name == "effort" && using_effort) {
      command_interfaces_.at(i).set_value((*joint_commands)->effort.at(command_index));
    } else if (interface_name == "k_q_p" && using_k_q_p) {
      command_interfaces_.at(i).set_value((*joint_commands)->k_q_p.at(command_index));
    } else if (interface_name == "k_qd_p" && using_k_qd_p) {
      command_interfaces_.at(i).set_value((*joint_commands)->k_qd_p.at(command_index));
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::SpotJointController, controller_interface::ControllerInterface)
