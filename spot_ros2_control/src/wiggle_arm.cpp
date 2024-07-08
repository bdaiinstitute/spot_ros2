// Based off: https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/wiggle_arm.py

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class WiggleArm : public rclcpp::Node {
 public:
  WiggleArm() : Node("wiggle_arm"), state{0}, initialized{false} {
    declare_parameter("timer_rate", 5.0);
    const auto timer_rate = std::chrono::duration<double>{get_parameter("timer_rate").as_double()};
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&WiggleArm::joint_states_callback, this, std::placeholders::_1));
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    timer_ = create_wall_timer(timer_rate, std::bind(&WiggleArm::timer_callback, this));
  }

 private:
  std_msgs::msg::Float64MultiArray command_start;
  std_msgs::msg::Float64MultiArray command_wiggle_down;
  std_msgs::msg::Float64MultiArray command_wiggle_up;
  int state;
  bool initialized;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    if (!initialized) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      // This assumes the robot has an arm+gripper (19 joints)
      command_start.data = msg.position;
      command_wiggle_down.data = msg.position;
      command_wiggle_down.data.at(16) += 1.0;
      command_wiggle_down.data.at(18) -= 0.5;
      command_wiggle_up.data = msg.position;
      command_wiggle_up.data.at(16) -= 1.0;
      command_wiggle_up.data.at(18) -= 0.5;
      initialized = true;
    }
  }

  void timer_callback() {
    if (!initialized) {
      return;
    }
    if (state == 0) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Wiggle Down");
      command_pub_->publish(command_wiggle_down);
    } else if (state == 1) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Start");
      command_pub_->publish(command_start);
    } else if (state == 2) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Wiggle Up");
      command_pub_->publish(command_wiggle_up);
    } else if (state == 3) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Start");
      command_pub_->publish(command_start);
    }
    state += 1;
    if (state >= 4) {
      state = 0;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WiggleArm>());
  rclcpp::shutdown();
  return 0;
}
