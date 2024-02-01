// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_tf_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver_cpp/robot_state/spot_robot_state_publisher.hpp>
#include <spot_driver_cpp/types.hpp>
#include <tl_expected/expected.hpp>

#include <memory>

namespace spot_ros2 {

/**
 * @brief Production implementation of a SpotRobotStatePublisher::MiddlewareHandle
 */

class RobotMiddlewareHandle : public SpotRobotStatePublisher::MiddlewareHandle {
 public:
  /**
   * @brief Constructor for RobotMiddlewareHandle.
   *
   * @param node A shared instance to a rclcpp::node
   */
  explicit RobotMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Delegating constructor for RobotMiddlewareHandle.
   *
   * @param node_options configuration options for a rclcpp::node
   */
  explicit RobotMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  ~RobotMiddlewareHandle() override = default;

  /**
   * @brief Create ROS publishers for Robot State
   */
  void createPublishers() override;

  /**
   * @brief Publish robot state messages
   */
  void publishRobotState(const RobotState& robot_state) override;

  ParameterInterfaceBase* parameter_interface() override { return parameter_interface_.get(); }
  LoggerInterfaceBase* logger_interface() override { return logger_interface_.get(); }
  TfInterfaceBase* tf_interface() override { return tf_interface_.get(); }
  TimerInterfaceBase* timer_interface() override { return timer_interface_.get(); }
  std::shared_ptr<rclcpp::Node> node() override { return node_; }

 private:
  /** @brief Shared instance of an rclcpp node to create publishers */
  std::shared_ptr<rclcpp::Node> node_;
  /** @brief instance of ParameterInterfaceBase to get ROS parameters*/
  std::unique_ptr<RclcppParameterInterface> parameter_interface_;
  /** @brief instance of LoggerInterfaceBase to send ROS log messages*/
  std::unique_ptr<RclcppLoggerInterface> logger_interface_;
  /** @brief instance of TfInterfaceBase to update static transforms*/
  std::unique_ptr<RclcppTfInterface> tf_interface_;
  /** @brief instance of TimerInterfaceBase to create callback timer*/
  std::unique_ptr<RclcppWallTimerInterface> timer_interface_;

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
