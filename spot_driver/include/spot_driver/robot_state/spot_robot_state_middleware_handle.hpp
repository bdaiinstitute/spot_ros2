// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rmw/qos_profiles.h>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <spot_driver/robot_state/spot_robot_state_publisher.hpp>
#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>

#include <memory>

namespace spot_ros2 {

/**
 * @brief Production implementation of a SpotRobotStatePublisher::MiddlewareHandle
 */

class SpotRobotStateMiddlewareHandle : public SpotRobotStatePublisher::MiddlewareHandle {
 public:
  /**
   * @brief Constructor for SpotRobotStateMiddlewareHandle.
   *
   * @param node A shared instance to a rclcpp::node
   */
  explicit SpotRobotStateMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Delegating constructor for SpotRobotStateMiddlewareHandle.
   *
   * @param node_options configuration options for a rclcpp::node
   */
  explicit SpotRobotStateMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  ~SpotRobotStateMiddlewareHandle() override = default;

  /**
   * @brief Publish robot state messages
   */
  void publishRobotState(const RobotState& robot_state) override;

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
  std::shared_ptr<rclcpp::Publisher<bosdyn_msgs::msg::ManipulatorState>> manipulator_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>> end_effector_force_publisher_;
  std::shared_ptr<rclcpp::Publisher<spot_msgs::msg::BehaviorFaultState>> behavior_fault_state_publisher_;
};

}  // namespace spot_ros2
