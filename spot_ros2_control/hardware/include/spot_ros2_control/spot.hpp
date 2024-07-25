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

#ifndef SPOT_ROS2_CONTROL__SPOT_HPP_
#define SPOT_ROS2_CONTROL__SPOT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "spot_ros2_control/visibility_control.h"

#include "bosdyn/client/lease/lease_keepalive.h"
#include "bosdyn/client/robot_command/robot_command_client.h"
#include "bosdyn/client/robot_command/robot_command_helpers.h"
#include "bosdyn/client/robot_command/robot_command_streaming_client.h"
#include "bosdyn/client/robot_state/robot_state_client.h"
#include "bosdyn/client/robot_state/robot_state_streaming_client.h"
#include "bosdyn/client/sdk/client_sdk.h"
#include "bosdyn/client/service_client/client_header_handling.h"
#include "bosdyn/client/time_sync/time_sync_helpers.h"
#include "bosdyn/client/util/cli_util.h"

using StateHandler = std::function<void(::bosdyn::api::RobotStateStreamResponse&)>;

namespace spot_ros2_control {

class StateStreamingHandler {
 public:
  void handle_state_streaming(::bosdyn::api::RobotStateStreamResponse& robot_state);
  std::vector<float> get_position();
  std::vector<float> get_velocity();
  std::vector<float> get_load();

 private:
  std::vector<float> current_position_;
  std::vector<float> current_velocity_;
  std::vector<float> current_load_;
};
class SpotHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SpotHardware)

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SPOT_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // TODO(khughes): Hard coding this for now, but there should be a cleaner way to do this.
  // The 3 interfaces are position, velocity, and effort.
  int interfaces_per_joint_ = 3;

  std::unique_ptr<::bosdyn::client::Robot> robot_;
  ::bosdyn::client::LeaseClient* lease_client_;
  ::bosdyn::client::RobotStateStreamingClient* state_client_;

  std::jthread state_thread_;
  StateStreamingHandler state_streaming_handler_;

  bool authenticate_robot(const std::string hostname, const std::string usernmame, const std::string password);
  bool start_time_sync();
  bool check_estop();
  bool get_lease();
  bool power_on();
  bool start_state_stream(StateHandler&& state_policy);
  void stop_state_stream();
  void release_lease();

  // Store the commands and states for the robot.
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace spot_ros2_control

#endif  // SPOT_ROS2_CONTROL__SPOT_HPP_
