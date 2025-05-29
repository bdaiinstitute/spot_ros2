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

#include "spot_controllers/spot_imu_broadcaster.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace spot_controllers {
SpotIMUBroadcaster::SpotIMUBroadcaster() : imu_sensor_broadcaster::IMUSensorBroadcaster() {}

controller_interface::CallbackReturn SpotIMUBroadcaster::on_init() {
  if (!get_node()) {
    fprintf(stderr, "Failed to aquire node handle!\n");
    return CallbackReturn::ERROR;
  }
  auto param_listener = std::make_shared<ParamListener>(get_node());
  params_ = param_listener->get_params();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotIMUBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::string prefix = "";
  if (params_.use_namespace_as_prefix) {
    prefix = get_prefix_from_namespace(get_node()->get_namespace());
  }

  RCLCPP_ERROR(get_node()->get_logger(), "Prefix: %s \n", prefix.c_str());

  imu_sensor_ =
      std::make_unique<semantic_components::IMUSensor>(semantic_components::IMUSensor(prefix + params_.sensor_name));
  try {
    // register ft sensor data publisher
    sensor_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during publisher creation at configure stage with message : %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = prefix + params_.frame_id;
  // convert double vector to fixed-size array in the message
  for (size_t i = 0; i < 9; ++i) {
    realtime_publisher_->msg_.orientation_covariance[i] = params_.static_covariance_orientation[i];
    realtime_publisher_->msg_.angular_velocity_covariance[i] = params_.static_covariance_angular_velocity[i];
    realtime_publisher_->msg_.linear_acceleration_covariance[i] = params_.static_covariance_linear_acceleration[i];
  }
  realtime_publisher_->unlock();

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::SpotIMUBroadcaster, controller_interface::ControllerInterface)
