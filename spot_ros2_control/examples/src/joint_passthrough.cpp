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

class JointPassthrough : public rclcpp::Node {
 public:
  JointPassthrough() : Node("joint_passthrough") {
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "Ethernet0/joint_states", 10, std::bind(&JointPassthrough::joint_states_callback, this, std::placeholders::_1));

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("test/commands", 10);
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

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  /// @brief Callback for receiving joint states messages used to store the nominal joint angles of the robot
  /// @param msg ROS message containing joint states
  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    int n_dof = msg.name.size();

    std_msgs::msg::Float64MultiArray spot_command;
    spot_command.data.resize(n_dof);

    for (int i = 0; i < n_dof; ++i) {
      std::string joint_name = msg.name[i].substr(msg.name[i].find("/") + 1);
      if (spot_joint_index_.find(joint_name) == spot_joint_index_.end()) {
        RCLCPP_ERROR(get_logger(), "Joint %s not found in spot joint map", msg.name[i].c_str());
        return;
      }

      int joint_idx = spot_joint_index_[joint_name];
      spot_command.data[joint_idx] = msg.position[i];
      RCLCPP_INFO(get_logger(), "Joint %s at index %i - %f", joint_name.c_str(), joint_idx, msg.position[i]);
    }

    std::cout << "Command: " << spot_command.data.size() << std::endl;
    for (const auto& joint : spot_command.data) {
      std::cout << joint << std::endl;
    }

    command_pub_->publish(spot_command);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPassthrough>());
  rclcpp::shutdown();
  return 0;
}
