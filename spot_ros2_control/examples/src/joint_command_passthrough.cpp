// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

// Based off:
// https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/wiggle_arm_example.cpp

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <ranges>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointCommandPassthrough : public rclcpp::Node {
 public:
  JointCommandPassthrough() : Node("joint_passthrough") {
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "Ethernet0/joint_states", 10, std::bind(&JointCommandPassthrough::joint_states_callback, this, std::placeholders::_1));

    joint_commands_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "Ethernet0/joint_commands", 10,
        std::bind(&JointCommandPassthrough::joint_commands_callback, this, std::placeholders::_1));

    command_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/Ethernet0/forward_position_controller/commands", 10);

    spot_command_.data.reserve(n_dof_);
  }

  // From https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#bosdyn-api-spot-JointIndex
  // This is the order the commands need to be sent in
  std::map<std::string, int> spot_joint_index_ = {
      {"front_left_hip_x", 0},  {"front_left_hip_y", 1}, {"front_left_knee", 2},   {"front_right_hip_x", 3},
      {"front_right_hip_y", 4}, {"front_right_knee", 5}, {"rear_left_hip_x", 6},   {"rear_left_hip_y", 7},
      {"rear_left_knee", 8},    {"rear_right_hip_x", 9}, {"rear_right_hip_y", 10}, {"rear_right_knee", 11},
      {"arm_sh0", 12},          {"arm_sh1", 13},         {"arm_el0", 14},          {"arm_el1", 15},
      {"arm_wr0", 16},          {"arm_wr1", 17},         {"arm_f1x", 18},
  };
  int n_dof_ = 19;

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  std_msgs::msg::Float64MultiArray spot_command_;

  int count_ = 0;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    // Keep track of the current joint positions
    if (msg.name.size() != static_cast<std::vector<int>::size_type>(n_dof_)) {
      RCLCPP_ERROR(get_logger(), "Expected %i dofs, but got %li", n_dof_, msg.name.size());
      return;
    }

    spot_command_.data.clear();
    spot_command_.data.resize(n_dof_);

    for (int i = 0; i < n_dof_; ++i) {
      std::string joint_name = msg.name[i].substr(msg.name[i].find("/") + 1);
      if (spot_joint_index_.find(joint_name) == spot_joint_index_.end()) {
        RCLCPP_ERROR(get_logger(), "Joint %s not found in spot joint map", msg.name[i].c_str());
        return;
      }

      int joint_idx = spot_joint_index_[joint_name];
      spot_command_.data[joint_idx] = msg.position[i];
      // RCLCPP_INFO(get_logger(), "Joint %s at index %i - %f", joint_name.c_str(), joint_idx, msg.position[i]);
    }
  }

  void joint_commands_callback(const sensor_msgs::msg::JointState& msg) {
    // Update provided joints with their desired

    int command_dofs = msg.name.size();

    for (int i = 0; i < command_dofs; ++i) {
      std::string joint_name = msg.name[i].substr(msg.name[i].find("/") + 1);
      if (spot_joint_index_.find(joint_name) == spot_joint_index_.end()) {
        RCLCPP_ERROR(get_logger(), "Joint %s not found in spot joint map", msg.name[i].c_str());
        return;
      }

      int joint_idx = spot_joint_index_[joint_name];
      spot_command_.data[joint_idx] = msg.position[i];
    }

    count_++;
    RCLCPP_INFO(get_logger(), "Forwarding command %i", count_);

    command_pub_->publish(spot_command_);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandPassthrough>());
  rclcpp::shutdown();
  return 0;
}
