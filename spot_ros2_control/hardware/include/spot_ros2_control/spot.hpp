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

#include <memory>
#include <string>
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

namespace spot_ros2_control {
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
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  SPOT_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  int interfaces_per_joint_;

  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  ::bosdyn::client::LeaseClient* lease_client;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace spot_ros2_control

#endif  // SPOT_ROS2_CONTROL__SPOT_HPP_
