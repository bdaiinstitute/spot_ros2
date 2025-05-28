// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 PAL Robotics S.L.
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

#include <spot_controllers/spot_imu_broadcaster_parameters.hpp>
#include "imu_sensor_broadcaster/imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"
#include "spot_controllers/spot_controller_utils.hpp"
#include "spot_controllers/visibility_control.h"

namespace spot_controllers {
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
class SpotIMUBroadcaster : public imu_sensor_broadcaster::IMUSensorBroadcaster {
 public:
  SPOT_CONTROLLERS_PUBLIC
  SpotIMUBroadcaster();

 protected:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  using Params = spot_imu_broadcaster::Params;
  using ParamListener = spot_imu_broadcaster::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace spot_controllers
