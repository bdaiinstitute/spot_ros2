// File modified. Modifications Copyright (c) 2025 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2021 PAL Robotics SL.
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
 * \brief Broadcasts IMU data from Spot.
 * This controller is largely taken from the IMU sensor broadcasters from the ros2_controllers repo:
 * https://github.com/ros-controls/ros2_controllers/tree/humble/imu_sensor_broadcaster
 *
 * \param use_namespace_as_prefix Flag to determine if the prefix for the IMU sensor name should be picked up
 * from the namespace that the controller is launched in.
 * \param sensor_name Sensor name used as prefix for its interfaces. Interface names are
 * Interface names are: ``<sensor_name>/orientation.x, ..., <sensor_name>/angular_velocity.x, ...,
 * <sensor_name>/linear_acceleration.x.``",
 * \param frame_id Frame ID used in the imu message
 * \param static_covariance_orientation Static orientation covariance. Row major about x, y, z axes
 * \param static_covariance_angular_velocity Static angular velocity covariance. Row major about x, y, z axes
 * \param static_covariance_linear_acceleration Static linear acceleration covariance. Row major about x, y, z axes
 *
 * Publishes to:
 * - \b imu (sensor_msgs::msg::Imu) : IMU sensor data from Spot
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
