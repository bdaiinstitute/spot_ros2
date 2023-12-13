// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot_state/robot_state_client.h>
#include <bosdyn_msgs/msg/manipulator_state.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tl_expected/expected.hpp>

#include <optional>
#include <string>

namespace spot_ros2 {

/**
 * @brief A struct of ROS message types that define Spot's Robot State
 */
struct RobotState {
  spot_msgs::msg::BatteryStateArray battery_states;
  spot_msgs::msg::WiFiState wifi_state;
  spot_msgs::msg::FootStateArray foot_state;
  spot_msgs::msg::EStopStateArray estop_states;

  std::optional<sensor_msgs::msg::JointState> maybe_joint_states;
  std::optional<tf2_msgs::msg::TFMessage> maybe_tf;
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> maybe_odom_twist;
  std::optional<nav_msgs::msg::Odometry> maybe_odom;
  std::optional<spot_msgs::msg::PowerState> maybe_power_state;
  std::optional<spot_msgs::msg::SystemFaultState> maybe_system_fault_state;
  std::optional<bosdyn_msgs::msg::ManipulatorState> maybe_manipulator_state;
  std::optional<geometry_msgs::msg::Vector3Stamped> maybe_end_effector_force;
  std::optional<spot_msgs::msg::BehaviorFaultState> maybe_behavior_fault_state;
};

/**
 * @brief Interface class to interact with Spot SDK's Robot State Client
 */
class RobotStateClientInterface {
 public:
  virtual tl::expected<RobotState, std::string> getRobotState(const std::string& preferred_odom_frame) = 0;
};
}  // namespace spot_ros2
