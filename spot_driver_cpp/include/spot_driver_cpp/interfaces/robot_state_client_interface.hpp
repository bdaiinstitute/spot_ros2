// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <bosdyn_msgs/msg/manipulation_feedback_state.hpp>
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
  spot_msgs::msg::FootState foot_state;
  spot_msgs::msg::EStopStateArray estop_states;
  
  std::optional<sensor_msgs::msg::JointState> maybe_joint_states;
  std::optional<tf2_msgs::msg::TFMessage> tf;
  std::optional<geometry_msgs::msg::TwistWithCovariance> odom_twist;
  std::optional<nav_msgs::msg::Odometry> odom;
  std::optional<spot_msgs::msg::PowerState> maybe_power_state;
  std::optional<spot_msgs::msg::SystemFaultState> maybe_system_fault_state;
  std::optional<bosdyn_msgs::msg::ManipulationFeedbackState> maybe_manipulation_state;
  std::optional<geometry_msgs::msg::Vector3Stamped> maybe_end_effector_force;
  std::optional<spot_msgs::msg::BehaviorFaultState> maybe_behavior_fault_state;
};

/**
 * @brief Interface class to interact with Spot SDK's Robot State Client
*/
class RobotStateClientInterface {
public:
  virtual tl::expected<RobotState, std::string> getRobotState(::bosdyn::api::GetRobotStateRequest request) = 0;
};
} // namespace spot_ros2
