// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/robot_state/state_middleware_handle.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spot_driver/types.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 1;
constexpr auto kNodeName{"spot_state_publisher"};

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

}  // namespace

namespace spot_ros2 {

StateMiddlewareHandle::StateMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node)
    : node_{node},
      battery_states_publisher_{node_->create_publisher<spot_msgs::msg::BatteryStateArray>(
          kBatteryStatesTopic, makePublisherQoS(kPublisherHistoryDepth))},
      wifi_state_publisher_{
          node_->create_publisher<spot_msgs::msg::WiFiState>(kWifiTopic, makePublisherQoS(kPublisherHistoryDepth))},
      foot_states_publisher_{node_->create_publisher<spot_msgs::msg::FootStateArray>(
          kFeetTopic, makePublisherQoS(kPublisherHistoryDepth))},
      estop_states_publisher_{node_->create_publisher<spot_msgs::msg::EStopStateArray>(
          kEStopTopic, makePublisherQoS(kPublisherHistoryDepth))},
      joint_state_publisher_{node_->create_publisher<sensor_msgs::msg::JointState>(
          kJointStatesTopic, makePublisherQoS(kPublisherHistoryDepth))},
      odom_twist_publisher_{node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          kOdomTwistTopic, makePublisherQoS(kPublisherHistoryDepth))},
      odom_publisher_{
          node_->create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, makePublisherQoS(kPublisherHistoryDepth))},
      power_state_publisher_{node_->create_publisher<spot_msgs::msg::PowerState>(
          kPowerStatesTopic, makePublisherQoS(kPublisherHistoryDepth))},
      system_faults_publisher_{node_->create_publisher<spot_msgs::msg::SystemFaultState>(
          kSystemFaultsTopic, makePublisherQoS(kPublisherHistoryDepth))},
      manipulator_state_publisher_{node_->create_publisher<bosdyn_api_msgs::msg::ManipulatorState>(
          kManipulatorTopic, makePublisherQoS(kPublisherHistoryDepth))},
      end_effector_force_publisher_{node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
          kEndEffectorForceTopic, makePublisherQoS(kPublisherHistoryDepth))},
      behavior_fault_state_publisher_{node_->create_publisher<spot_msgs::msg::BehaviorFaultState>(
          kBehaviorFaultsTopic, makePublisherQoS(kPublisherHistoryDepth))} {}

StateMiddlewareHandle::StateMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : StateMiddlewareHandle(std::make_shared<rclcpp::Node>(kNodeName, node_options)) {}

void StateMiddlewareHandle::publishRobotState(const RobotStateMessages& robot_state_msgs) {
  battery_states_publisher_->publish(robot_state_msgs.battery_states);
  wifi_state_publisher_->publish(robot_state_msgs.wifi_state);
  foot_states_publisher_->publish(robot_state_msgs.foot_state);
  estop_states_publisher_->publish(robot_state_msgs.estop_states);

  if (robot_state_msgs.maybe_joint_states) {
    joint_state_publisher_->publish(robot_state_msgs.maybe_joint_states.value());
  }
  if (robot_state_msgs.maybe_odom_twist) {
    odom_twist_publisher_->publish(robot_state_msgs.maybe_odom_twist.value());
  }
  if (robot_state_msgs.maybe_odom) {
    odom_publisher_->publish(robot_state_msgs.maybe_odom.value());
  }
  if (robot_state_msgs.maybe_power_state) {
    power_state_publisher_->publish(robot_state_msgs.maybe_power_state.value());
  }
  if (robot_state_msgs.maybe_system_fault_state) {
    system_faults_publisher_->publish(robot_state_msgs.maybe_system_fault_state.value());
  }
  if (robot_state_msgs.maybe_manipulator_state) {
    manipulator_state_publisher_->publish(robot_state_msgs.maybe_manipulator_state.value());
  }
  if (robot_state_msgs.maybe_end_effector_force) {
    end_effector_force_publisher_->publish(robot_state_msgs.maybe_end_effector_force.value());
  }
  if (robot_state_msgs.maybe_behavior_fault_state) {
    behavior_fault_state_publisher_->publish(robot_state_msgs.maybe_behavior_fault_state.value());
  }
}

}  // namespace spot_ros2
