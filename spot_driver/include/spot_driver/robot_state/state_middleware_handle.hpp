// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spot_driver/robot_state/state_publisher.hpp>
#include <spot_driver/types.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>

namespace spot_ros2 {

/**
 * @brief Production implementation of a StatePublisher::MiddlewareHandle
 */

class StateMiddlewareHandle : public StatePublisher::MiddlewareHandle {
 public:
  /**
   * @brief Constructor for StateMiddlewareHandle.
   * @param node A shared_pr to an instance of rclcpp::Node.
   */
  explicit StateMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Constructor for StateMiddlewareHandle.
   * @details This constructor creates a new rclcpp::Node using the provided NodeOptions.
   * @param node_options configuration options for a rclcpp::Node.
   */
  explicit StateMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  ~StateMiddlewareHandle() override = default;

  /**
   * @brief Publish robot state messages
   * @param robot_state_msgs Robot state messages to publish
   */
  void publishRobotState(const RobotStateMessages& robot_state_msgs) override;

 private:
  /** @brief Shared instance of an rclcpp node to create publishers */
  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::BatteryStateArray>> battery_states_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::WiFiState>> wifi_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::FootStateArray>> foot_states_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::EStopStateArray>> estop_states_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> odom_twist_publisher_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::PowerState>> power_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::SystemFaultState>> system_faults_publisher_;
  std::shared_ptr<rclcpp::Publisher<bosdyn_api_msgs::msg::ManipulatorState>> manipulator_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>> end_effector_force_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::BehaviorFaultState>> behavior_fault_state_publisher_;
};

}  // namespace spot_ros2
