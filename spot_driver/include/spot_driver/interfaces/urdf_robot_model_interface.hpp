// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <urdf_model/model.h>
#include <atomic>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <set>
#include <spot_driver/interfaces/robot_model_interface_base.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
/**
 * @brief Parse robot model data from a URDF published on the "robot_description" topic.
 */
class UrdfRobotModelInterface final : public RobotModelInterfaceBase {
 public:
  /**
   * @brief Constructor for UrdfRobotModelInterface.
   * @details Creates a rclcpp subscription on the robot's "robot_description" topic using transient local and reliable
   * QoS. This matches the QoS of the robot description topic advertised by the robot state publisher node.
   *
   * @param node rclcpp::Node used to create the subscription.
   */
  explicit UrdfRobotModelInterface(const std::shared_ptr<rclcpp::Node>& node);

  [[nodiscard]] tl::expected<std::set<std::string>, std::string> getFrameIds() const override;

 private:
  void onRobotDescription(const std_msgs::msg::String& msg);

  std::atomic_bool has_robot_model_;
  mutable std::mutex robot_model_mutex_;
  std::shared_ptr<urdf::ModelInterface> model_;

  std::shared_ptr<rclcpp::CallbackGroup> subscriber_callback_group_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> urdf_subscriber_;
};
}  // namespace spot_ros2
