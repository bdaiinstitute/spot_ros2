// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/robot_state/spot_robot_state_middleware_handle.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 1;
constexpr auto kNodeName{"spot_robot_state_publisher"};

// ROS topic names for Spot's robot state publisher
constexpr auto kJointStatesTopic{"joint_states"};
constexpr auto kOdomTwistTopic{"odometry/twist"};
constexpr auto kOdomTopic{"odometry"};
constexpr auto kFeetTopic{"status/feet"};
constexpr auto kEStopTopic{"status/estop"};
constexpr auto kWifiTopic{"status/wifi"};
constexpr auto kBatteryStatesTopic{"status/battery_states"};
constexpr auto kPowerStatesTopic{"status/power_states"};
constexpr auto kSystemFaultsTopic{"status/system_faults"};
constexpr auto kBehaviorFaultsTopic{"status/behavior_faults"};
constexpr auto kEndEffectorForceTopic{"status/end_effector_force"};
constexpr auto kManipulatorTopic{"manipulation_state"};

rclcpp::QoS makeQoS() {
  return rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth));
}
}  // namespace

namespace spot_ros2 {

SpotRobotStateMiddlewareHandle::SpotRobotStateMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node)
    : node_{node},
      battery_states_publisher_{
          node_->create_publisher<spot_msgs::msg::BatteryStateArray>(kBatteryStatesTopic, makeQoS())},
      wifi_state_publisher_{node_->create_publisher<spot_msgs::msg::WiFiState>(kWifiTopic, makeQoS())},
      foot_states_publisher_{node_->create_publisher<spot_msgs::msg::FootStateArray>(kFeetTopic, makeQoS())},
      estop_states_publisher_{node_->create_publisher<spot_msgs::msg::EStopStateArray>(kEStopTopic, makeQoS())},
      joint_state_publisher_{node_->create_publisher<sensor_msgs::msg::JointState>(kJointStatesTopic, makeQoS())},
      odom_twist_publisher_{
          node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(kOdomTwistTopic, makeQoS())},
      odom_publisher_{node_->create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, makeQoS())},
      power_state_publisher_{node_->create_publisher<spot_msgs::msg::PowerState>(kPowerStatesTopic, makeQoS())},
      system_faults_publisher_{
          node_->create_publisher<spot_msgs::msg::SystemFaultState>(kSystemFaultsTopic, makeQoS())},
      manipulator_state_publisher_{
          node_->create_publisher<bosdyn_msgs::msg::ManipulatorState>(kManipulatorTopic, makeQoS())},
      end_effector_force_publisher_{
          node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(kEndEffectorForceTopic, makeQoS())},
      behavior_fault_state_publisher_{
          node_->create_publisher<spot_msgs::msg::BehaviorFaultState>(kBehaviorFaultsTopic, makeQoS())} {}

SpotRobotStateMiddlewareHandle::SpotRobotStateMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : SpotRobotStateMiddlewareHandle(std::make_shared<rclcpp::Node>(kNodeName, node_options)) {}

void SpotRobotStateMiddlewareHandle::publishRobotState(const RobotState& robot_state) {
  battery_states_publisher_->publish(robot_state.battery_states);
  wifi_state_publisher_->publish(robot_state.wifi_state);
  foot_states_publisher_->publish(robot_state.foot_state);
  estop_states_publisher_->publish(robot_state.estop_states);

  if (robot_state.maybe_joint_states) {
    joint_state_publisher_->publish(robot_state.maybe_joint_states.value());
  }
  if (robot_state.maybe_odom_twist) {
    odom_twist_publisher_->publish(robot_state.maybe_odom_twist.value());
  }
  if (robot_state.maybe_odom) {
    odom_publisher_->publish(robot_state.maybe_odom.value());
  }
  if (robot_state.maybe_power_state) {
    power_state_publisher_->publish(robot_state.maybe_power_state.value());
  }
  if (robot_state.maybe_system_fault_state) {
    system_faults_publisher_->publish(robot_state.maybe_system_fault_state.value());
  }
  if (robot_state.maybe_manipulator_state) {
    manipulator_state_publisher_->publish(robot_state.maybe_manipulator_state.value());
  }
  if (robot_state.maybe_end_effector_force) {
    end_effector_force_publisher_->publish(robot_state.maybe_end_effector_force.value());
  }
  if (robot_state.maybe_behavior_fault_state) {
    behavior_fault_state_publisher_->publish(robot_state.maybe_behavior_fault_state.value());
  }
}

}  // namespace spot_ros2
