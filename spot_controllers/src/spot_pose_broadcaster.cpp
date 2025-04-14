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
    state_interfaces_config.names.emplace_back(interface);
  }
  for (const auto& interface : odom_pose_sensor_->get_state_interface_names()) {
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
  tf_publish_period_ = params_.tf.publish_rate == 0.0
                           ? std::nullopt
                           : std::optional{rclcpp::Duration::from_seconds(1.0 / params_.tf.publish_rate)};

  try {
    pose_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(DEFAULT_POSE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(pose_publisher_);

    if (params_.tf.enable) {
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
  realtime_publisher_->msg_.header.frame_id = params_.frame_id;
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
  // FIXME this seems wrong, each sensor has a different set of state interfaces
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
  geometry_msgs::msg::Pose vision_t_body;
  vision_pose_sensor_->get_values_as_message(vision_t_body);

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
    bool do_publish = false;
    // rlcpp::Time comparisons throw if clock types are not the same
    if (tf_last_publish_time_.get_clock_type() != time.get_clock_type()) {
      do_publish = true;
    } else if (!tf_publish_period_ || (tf_last_publish_time_ + *tf_publish_period_ <= time)) {
      do_publish = true;
    }

    if (do_publish) {
      auto& vision_tf_transform = realtime_tf_publisher_->msg_.transforms[0];
      vision_tf_transform.header.stamp = time;

      vision_tf_transform.transform.translation.x = vision_t_body.position.x;
      vision_tf_transform.transform.translation.y = vision_t_body.position.y;
      vision_tf_transform.transform.translation.z = vision_t_body.position.z;

      vision_tf_transform.transform.rotation.x = vision_t_body.orientation.x;
      vision_tf_transform.transform.rotation.y = vision_t_body.orientation.y;
      vision_tf_transform.transform.rotation.z = vision_t_body.orientation.z;
      vision_tf_transform.transform.rotation.w = vision_t_body.orientation.w;

      auto& odom_tf_transform = realtime_tf_publisher_->msg_.transforms[1];
      odom_tf_transform.header.stamp = time;

      odom_tf_transform.transform.translation.x = vision_t_body.position.x;
      odom_tf_transform.transform.translation.y = vision_t_body.position.y;
      odom_tf_transform.transform.translation.z = vision_t_body.position.z;

      odom_tf_transform.transform.rotation.x = vision_t_body.orientation.x;
      odom_tf_transform.transform.rotation.y = vision_t_body.orientation.y;
      odom_tf_transform.transform.rotation.z = vision_t_body.orientation.z;
      odom_tf_transform.transform.rotation.w = vision_t_body.orientation.w;

      realtime_tf_publisher_->unlockAndPublish();

      tf_last_publish_time_ = time;
    } else {
      realtime_tf_publisher_->unlock();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::SpotPoseBroadcaster, controller_interface::ControllerInterface)
