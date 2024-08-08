// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

// Based off: https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/noarm_squat.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum class SquatState { INITIALIZING, SQUATTING, STANDING };
class NoarmSquat : public rclcpp::Node {
 public:
  NoarmSquat()
      : Node("noarm_squat"), squat_state_{SquatState::INITIALIZING}, initialized_{false}, count_{0}, njoints_{12} {
    stand_joint_angles_ = declare_parameter("stand_joint_angles", std::vector<double>{});
    squat_joint_angles_ = declare_parameter("squat_joint_angles", std::vector<double>{});
    const auto command_rate = declare_parameter("command_rate", 50.0);  // how frequently to send commands in Hz
    const auto seconds_per_motion =
        declare_parameter("seconds_per_motion", 2.0);  // how many seconds the squat and stand motions should take

    if (stand_joint_angles_.size() != njoints_) {
      throw std::logic_error("Stand joint angles is the wrong size!");
    }
    if (squat_joint_angles_.size() != njoints_) {
      throw std::logic_error("Squat joint angles is the wrong size!");
    }

    for (size_t i = 0; i < njoints_; i++) {
      diff_squat_stand_.push_back(squat_joint_angles_.at(i) - stand_joint_angles_.at(i));
    }

    points_per_motion_ = static_cast<int>(command_rate * seconds_per_motion);
    command_.data = std::vector<double>(njoints_, 0.0);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&NoarmSquat::joint_states_callback, this, std::placeholders::_1));

    const auto timer_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1. / command_rate));
    timer_ = create_wall_timer(timer_period, std::bind(&NoarmSquat::timer_callback, this));
  }

 private:
  // For storing joint angles
  std::vector<double> stand_joint_angles_;
  std::vector<double> squat_joint_angles_;
  std::vector<double> init_joint_angles_;
  // For storing differences relevant for calculating the interpolation
  std::vector<double> diff_squat_stand_;
  std::vector<double> diff_squat_init_;
  // Command we send to the robot
  std_msgs::msg::Float64MultiArray command_;
  // Parameters and info about state
  SquatState squat_state_;
  int points_per_motion_;
  bool initialized_;
  int count_;
  size_t njoints_;
  // RCLCPP publishers, subscribers, timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    // Used to save the starting joint angles
    if (!initialized_) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      init_joint_angles_ = msg.position;
      for (size_t i = 0; i < njoints_; i++) {
        diff_squat_init_.push_back(squat_joint_angles_.at(i) - init_joint_angles_.at(i));
      }
      initialized_ = true;
    }
  }

  void timer_callback() {
    // Wait to send commands until we have initialized with the starting joint angles
    if (!initialized_) {
      return;
    }
    // Check if we need to switch state
    if (count_ > points_per_motion_) {
      switch (squat_state_) {
        case SquatState::INITIALIZING:
          squat_state_ = SquatState::SQUATTING;
          break;
        case SquatState::SQUATTING:
          squat_state_ = SquatState::STANDING;
          break;
        case SquatState::STANDING:
          squat_state_ = SquatState::SQUATTING;
          break;
      }
      count_ = 0;
    }
    // Percentage we are through the desired motion
    const double percentage = static_cast<float>(count_) / points_per_motion_;
    // Fill in the command with the appropriate joint angles given the state
    switch (squat_state_) {
      case SquatState::INITIALIZING:
        for (size_t i = 0; i < njoints_; i++) {
          command_.data.at(i) = percentage * diff_squat_init_.at(i) + init_joint_angles_.at(i);
        }
        break;
      case SquatState::SQUATTING:
        for (size_t i = 0; i < njoints_; i++) {
          command_.data.at(i) = percentage * -diff_squat_stand_.at(i) + squat_joint_angles_.at(i);
        }
        break;
      case SquatState::STANDING:
        for (size_t i = 0; i < njoints_; i++) {
          command_.data.at(i) = percentage * diff_squat_stand_.at(i) + stand_joint_angles_.at(i);
        }
        break;
    }
    // Publish the command and increment count
    command_pub_->publish(command_);
    count_++;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoarmSquat>());
  rclcpp::shutdown();
  return 0;
}
