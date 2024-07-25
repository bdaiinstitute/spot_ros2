// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 ros2_control Development Team
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

#include "spot_ros2_control/spot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace spot_ros2_control {

void StateStreamingHandler::handle_state_streaming(::bosdyn::api::RobotStateStreamResponse& robot_state) {
  auto& position_msg = robot_state.joint_states().position();
  auto& velocity_msg = robot_state.joint_states().velocity();
  auto& load_msg = robot_state.joint_states().load();

  // order is:
  // leg: fl hip x, fl hip y, fl knee, fr..., rl...., rr...,
  // arm: sh0, sh1, el0, el1, wr0, wr1, f1x

  current_position_ = {position_msg.begin(), position_msg.end()};
  current_velocity_ = {velocity_msg.begin(), velocity_msg.end()};
  current_load_ = {load_msg.begin(), load_msg.end()};
}

std::vector<float> StateStreamingHandler::get_position() {
  return current_position_;
}
std::vector<float> StateStreamingHandler::get_velocity() {
  return current_velocity_;
}
std::vector<float> StateStreamingHandler::get_load() {
  return current_load_;
}

hardware_interface::CallbackReturn SpotHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto hostname = info_.hardware_parameters["hostname"];
  const auto username = info_.hardware_parameters["username"];
  const auto password = info_.hardware_parameters["password"];
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size() * interfaces_per_joint_, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size() * interfaces_per_joint_, std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // Assumes three state and three command interfaces for each joint (position, velocity, and effort).
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' has %zu command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Correct number of joint interfaces!");

  // Set up the robot using the BD SDK
  if (!authenticate_robot(hostname, username, password)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!start_time_sync()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!check_estop()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!get_lease()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!power_on()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!start_state_stream(std::bind(&StateStreamingHandler::handle_state_streaming, &state_streaming_handler_,
                                    std::placeholders::_1))) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++) {
    // RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "hw states %d", i);
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SpotHardware::export_state_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Export state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto joint = info_.joints.at(i);
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION,
                                                                     &hw_states_[interfaces_per_joint_ * i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY,
                                                                     &hw_states_[interfaces_per_joint_ * i + 1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT,
                                                                     &hw_states_[interfaces_per_joint_ * i + 2]));
  }

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Return state interfaces");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SpotHardware::export_command_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Export command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // info_.joints[i].state_interfaces[0].name
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto joint = info_.joints.at(i);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION,
                                                                         &hw_commands_[interfaces_per_joint_ * i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY,
                                                                         &hw_commands_[interfaces_per_joint_ * i + 1]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT,
                                                                         &hw_commands_[interfaces_per_joint_ * i + 2]));
  }

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Return command interfaces");

  return command_interfaces;
}

hardware_interface::CallbackReturn SpotHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Deactivating!");
  stop_state_stream();
  release_lease();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Shutting down");
  stop_state_stream();
  release_lease();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SpotHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const auto joint_pos = state_streaming_handler_.get_position();
  const auto joint_vel = state_streaming_handler_.get_velocity();
  const auto joint_load = state_streaming_handler_.get_load();
  // wait for them to be initialized
  if (joint_pos.size() == 0 || joint_vel.size() == 0 || joint_load.size() == 0) {
    return hardware_interface::return_type::OK;
  }
  // read into joint states
  for (uint i = 0; i < hw_states_.size(); i += 3) {
    hw_states_.at(i) = joint_pos.at(i / 3);
    hw_states_.at(i + 1) = joint_vel.at(i / 3);
    hw_states_.at(i + 2) = joint_load.at(i / 3);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SpotHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Writing...");

  // for (uint i = 0; i < hw_commands_.size(); i++) {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Got command %.5f for joint %d!", hw_commands_[i], i);
  // }
  // RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Joints successfully written!");
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

bool SpotHardware::authenticate_robot(const std::string hostname, const std::string username,
                                      const std::string password) {
  // Create a Client SDK object.
  auto client_sdk_ = ::bosdyn::client::CreateStandardSDK("SpotHardware");
  auto robot_result = client_sdk_->CreateRobot(hostname);
  if (!robot_result) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not create robot");
    return false;
  }
  robot_ = robot_result.move();
  ::bosdyn::common::Status status = robot_->Authenticate(username, password);
  if (!status) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"),
                "Could not authenticate robot with hostname: %s username: %s password: %s", hostname.c_str(),
                username.c_str(), password.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot successfully authenticated!");
  return true;
}

bool SpotHardware::start_time_sync() {
  // Establish time synchronization with the robot
  ::bosdyn::client::Result<::bosdyn::client::TimeSyncClient*> time_sync_client_resp =
      robot_->EnsureServiceClient<::bosdyn::client::TimeSyncClient>();
  if (!time_sync_client_resp.status) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not create time sync client");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Created time sync client");
  ::bosdyn::client::TimeSyncClient* time_sync_client = time_sync_client_resp.response;
  ::bosdyn::client::TimeSyncThread time_sync_thread(time_sync_client);
  if (time_sync_thread.HasEstablishedTimeSync()) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Faulty establishment of time sync");
    return false;
  }
  // Start time sync
  time_sync_thread.Start();
  if (!time_sync_thread.WaitForSync(std::chrono::seconds(5))) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Failed to establish time sync before timing out");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Time sync complete");
  return true;
}

bool SpotHardware::check_estop() {
  // Verify the robot is not estopped and that an external application has registered and holds
  // an estop endpoint.
  auto estop_status = robot_->IsEstopped();
  if (!estop_status) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not check estop status");
    return false;
  }
  if (estop_status.response) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot is e-stopped, cannot continue.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Estop check complete!");
  return true;
}

bool SpotHardware::get_lease() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Getting Lease");
  // First create a lease client.
  ::bosdyn::client::Result<::bosdyn::client::LeaseClient*> lease_client_resp =
      robot_->EnsureServiceClient<::bosdyn::client::LeaseClient>();
  if (!lease_client_resp) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not create lease client");
    return false;
  }
  lease_client_ = lease_client_resp.response;
  // Then acquire the lease for the body.
  auto lease_res = lease_client_->AcquireLease("body");
  if (!lease_res) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not acquire body lease");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Lease acquired!!");
  return true;
}

bool SpotHardware::power_on() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powering on...");
  auto power_status = robot_->PowerOnMotors(std::chrono::seconds(60), 1.0);
  if (!power_status) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not power on the robot");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powered on!");
  return true;
}

void state_stream_loop(std::stop_token stop_token, ::bosdyn::client::RobotStateStreamingClient* stateStreamClient,
                       StateHandler&& state_policy) {
  ::bosdyn::api::RobotStateStreamResponse latest_state_stream_response;

  while (!stop_token.stop_requested()) {
    // Get robot state stream
    auto robot_state_stream = stateStreamClient->GetRobotStateStream();
    if (!robot_state_stream) {
      RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Failed to get robot state");
      continue;
    }
    latest_state_stream_response = std::move(robot_state_stream.response);
    state_policy(latest_state_stream_response);
  }
}

bool SpotHardware::start_state_stream(StateHandler&& state_policy) {
  // Start state streaming
  auto robot_state_stream_client_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotStateStreamingClient>();
  if (!robot_state_stream_client_resp) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Could not create robot state client");
    return false;
  }
  state_client_ = robot_state_stream_client_resp.move();
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot State Client created");

  state_thread_ = std::jthread(&spot_ros2_control::state_stream_loop, state_client_, state_policy);
  return true;
}

void SpotHardware::stop_state_stream() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Stopping State Stream");
  state_thread_.request_stop();
  state_thread_.join();
}

void SpotHardware::release_lease() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Releasing Lease");
  bosdyn::api::ReturnLeaseRequest msg;
  auto lease_result = robot_->GetWallet()->GetOwnedLeaseProto("body");
  msg.mutable_lease()->CopyFrom(lease_result.response);
  auto resp = lease_client_->ReturnLease(msg);
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Return lease status: %s", resp.status.DebugString().c_str());
}

}  // namespace spot_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_ros2_control::SpotHardware, hardware_interface::SystemInterface)
