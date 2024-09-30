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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "forward_command_controller/forward_command_controller/forward_controllers_base.hpp"
#include "forward_state_controller_parameters.hpp"  // NOLINT(build/include_subdir)
#include "spot_ros2_control/visibility_control.h"

namespace spot_ros2_control {
/**
 * \brief Forward command controller for a set of interfaces.
 *
 * This class forwards the command signal for a set of interfaces over a set of joints.
 *
 * \param joints Names of the joint to control.
 * \param interface_names Names of the interfaces to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class ForwardStateController : public forward_command_controller::ForwardControllersBase {
 public:
  SPOT_ROS2_CONTROL_PUBLIC
  ForwardStateController();

 protected:
  void declare_parameters() override;
  controller_interface::CallbackReturn read_parameters() override;

  using Params = forward_state_controller::Params;
  using ParamListener = forward_state_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace spot_ros2_control
