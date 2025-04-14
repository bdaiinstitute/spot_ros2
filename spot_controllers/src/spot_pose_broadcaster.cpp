// Copyright 2024 FZI Forschungszentrum Informatik
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
#include "spot_controllers/spot_pose_broadcaster.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

namespace {

constexpr auto DEFAULT_POSE_TOPIC = "~/pose";
constexpr auto DEFAULT_TF_TOPIC = "/tf";

}  // namespace

namespace spot_controllers {

bool is_pose_valid(const geometry_msgs::msg::Pose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) &&

         std::abs(pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
                  pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w - 1.0) <= 10e-3;
}

controller_interface::InterfaceConfiguration SpotPoseBroadcaster::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SpotPoseBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& interface : vision_pose_sensor_->get_state_interface_names()) {
    RCLCPP_ERROR(get_node()->get_logger(), "vision interface %s", interface.c_str());
    state_interfaces_config.names.emplace_back(interface);
  }
  for (const auto& interface : odom_pose_sensor_->get_state_interface_names()) {
    RCLCPP_ERROR(get_node()->get_logger(), "odom interface %s", interface.c_str());
    state_interfaces_config.names.emplace_back(interface);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn SpotPoseBroadcaster::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& ex) {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotPoseBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();

  RCLCPP_ERROR(get_node()->get_logger(), "vision tform body %s", params_.vision_t_body.sensor_name.c_str());
  RCLCPP_ERROR(get_node()->get_logger(), "odom tform body %s", params_.odom_t_body.sensor_name.c_str());

  const bool use_namespace_as_prefix = params_.use_namespace_as_prefix;
  std::string frame_prefix = "";
  if (use_namespace_as_prefix) {
    // Grab the namespace from the node.
    const std::string node_namespace = get_node()->get_namespace();
    // namespace is "/" if there is none, else it's "/<namespace>". Erase the first char ("/") for easier parsing.
    const std::string trimmed_namespace = node_namespace.substr(1);
    // Now trimmed_namespace is either the empty string or "<namespace>"
    frame_prefix = trimmed_namespace.empty() ? "" : trimmed_namespace + "/";
    // And now sensor prefix is either "" or "<namespace>/"
  }
  RCLCPP_ERROR(get_node()->get_logger(), "frame prefix %s", frame_prefix.c_str());

  vision_pose_sensor_ =
      std::make_unique<semantic_components::PoseSensor>(frame_prefix + params_.vision_t_body.sensor_name);
  odom_pose_sensor_ = std::make_unique<semantic_components::PoseSensor>(frame_prefix + params_.odom_t_body.sensor_name);

  try {
    pose_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(DEFAULT_POSE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(pose_publisher_);

    if (params_.tf_enable) {
      tf_publisher_ =
          get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TF_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_tf_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_publisher_);
    }
  } catch (const std::exception& ex) {
    fprintf(stderr, "Exception thrown during publisher creation at configure stage with message: %s\n", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize pose message
  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = params_.vision_t_body.parent_frame_name;
  realtime_publisher_->unlock();

  // Initialize tf message if tf publishing is enabled
  if (realtime_tf_publisher_) {
    realtime_tf_publisher_->lock();

    realtime_tf_publisher_->msg_.transforms.resize(2);
    auto& vision_tf_transform = realtime_tf_publisher_->msg_.transforms.at(0);
    vision_tf_transform.header.frame_id = frame_prefix + params_.vision_t_body.parent_frame_name;
    vision_tf_transform.child_frame_id = frame_prefix + params_.vision_t_body.frame_name;

    auto& odom_tf_transform = realtime_tf_publisher_->msg_.transforms.at(1);
    odom_tf_transform.header.frame_id = frame_prefix + params_.odom_t_body.parent_frame_name;
    odom_tf_transform.child_frame_id = frame_prefix + params_.odom_t_body.frame_name;

    realtime_tf_publisher_->unlock();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotPoseBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  vision_pose_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  // FIXME this seems wrong, each sensor has a different set of state interfaces, but seems to work
  odom_pose_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotPoseBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  vision_pose_sensor_->release_interfaces();
  odom_pose_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpotPoseBroadcaster::update(const rclcpp::Time& time,
                                                              const rclcpp::Duration& /*period*/) {
  geometry_msgs::msg::Pose vision_t_body, odom_t_body;
  vision_pose_sensor_->get_values_as_message(vision_t_body);
  odom_pose_sensor_->get_values_as_message(odom_t_body);

  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.pose = vision_t_body;
    realtime_publisher_->unlockAndPublish();
  }
  if (!is_pose_valid(vision_t_body)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Invalid pose [%f, %f, %f], [%f, %f, %f, %f]", vision_t_body.position.x,
                          vision_t_body.position.y, vision_t_body.position.z, vision_t_body.orientation.x,
                          vision_t_body.orientation.y, vision_t_body.orientation.z, vision_t_body.orientation.w);
  } else if (realtime_tf_publisher_ && realtime_tf_publisher_->trylock()) {
    // FIXME consolidate logic
    const std::vector<geometry_msgs::msg::Pose> poses = {vision_t_body, odom_t_body};

    for (size_t i = 0; i < poses.size(); i++) {
      const auto& current_pose = poses.at(i);

      auto& tf_transform = realtime_tf_publisher_->msg_.transforms.at(i);
      tf_transform.header.stamp = time;

      tf_transform.transform.translation.x = current_pose.position.x;
      tf_transform.transform.translation.y = current_pose.position.y;
      tf_transform.transform.translation.z = current_pose.position.z;

      tf_transform.transform.rotation.x = current_pose.orientation.x;
      tf_transform.transform.rotation.y = current_pose.orientation.y;
      tf_transform.transform.rotation.z = current_pose.orientation.z;
      tf_transform.transform.rotation.w = current_pose.orientation.w;

      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Transform %ld, [%f, %f, %f], [%f, %f, %f, %f]", i, tf_transform.transform.translation.x,
                            tf_transform.transform.translation.y, tf_transform.transform.translation.z,
                            tf_transform.transform.rotation.x, tf_transform.transform.rotation.y,
                            tf_transform.transform.rotation.z, tf_transform.transform.rotation.w);
    }
    realtime_tf_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::SpotPoseBroadcaster, controller_interface::ControllerInterface)
