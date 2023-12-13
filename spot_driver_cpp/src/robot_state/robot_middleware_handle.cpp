// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/robot_state/robot_middleware_handle.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 1;
constexpr auto kNodeName{"spot_robot_state_publisher"};

// ROS topic names for Spot's robot state publisher
constexpr auto kJointStatesTopic{"joint_states"};
constexpr auto kTransformsTopic{"tf"};
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

RobotMiddlewareHandle::RobotMiddlewareHandle(std::shared_ptr<rclcpp::Node> node)
    : node_{node},
      parameter_interface_{std::make_unique<RclcppParameterInterface>(node)},
      logger_interface_{std::make_unique<RclcppLoggerInterface>(node->get_logger())},
      tf_interface_{std::make_unique<RclcppTfInterface>(node)},
      timer_interface_{std::make_unique<RclcppWallTimerInterface>(node)} {}

RobotMiddlewareHandle::RobotMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : node_{std::make_shared<rclcpp::Node>(kNodeName, node_options)} {}

void RobotMiddlewareHandle::createPublishers() {
  const auto topic_prefix = parameter_interface_->getSpotName() + "/";

  battery_states_publisher_ = node_->create_publisher<spot_msgs::msg::BatteryStateArray>(
      topic_prefix + kBatteryStatesTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  wifi_state_publisher_ = node_->create_publisher<spot_msgs::msg::WiFiState>(
      topic_prefix + kWifiTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  foot_states_publisher_ = node_->create_publisher<spot_msgs::msg::FootStateArray>(
      topic_prefix + kFeetTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  estop_states_publisher_ = node_->create_publisher<spot_msgs::msg::EStopStateArray>(
      topic_prefix + kEStopTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      topic_prefix + kJointStatesTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  dynamic_tf_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
      topic_prefix + kTransformsTopic,
      rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));  // TODO(abaker-bdai): maybe move to tf interface?
  odom_twist_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      topic_prefix + kOdomTwistTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      topic_prefix + kOdomTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  power_state_publisher_ = node_->create_publisher<spot_msgs::msg::PowerState>(
      topic_prefix + kPowerStatesTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  system_faults_publisher_ = node_->create_publisher<spot_msgs::msg::SystemFaultState>(
      topic_prefix + kSystemFaultsTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  manipulator_state_publisher_ = node_->create_publisher<bosdyn_msgs::msg::ManipulatorState>(
      topic_prefix + kManipulatorTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  end_effector_force_publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      topic_prefix + kEndEffectorForceTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
  behavior_fault_state_publisher_ = node_->create_publisher<spot_msgs::msg::BehaviorFaultState>(
      topic_prefix + kBehaviorFaultsTopic, rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth)));
}

void RobotMiddlewareHandle::publishRobotState(const RobotState& robot_state) {
  battery_states_publisher_->publish(robot_state.battery_states);
  wifi_state_publisher_->publish(robot_state.wifi_state);
  foot_states_publisher_->publish(robot_state.foot_state);
  estop_states_publisher_->publish(robot_state.estop_states);

  if (robot_state.maybe_joint_states) {
    joint_state_publisher_->publish(robot_state.maybe_joint_states.value());
  }
  if (robot_state.maybe_tf) {
    dynamic_tf_publisher_->publish(robot_state.maybe_tf.value());
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
