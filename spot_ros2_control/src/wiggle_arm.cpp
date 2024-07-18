// Based off: https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/wiggle_arm.py

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum WiggleState { START, WIGGLE_DOWN, WIGGLE_MIDDLE, WIGGLE_UP };

// TODO(khughes): make a struct or enum that maps joint name to ID number once we get more complicated examples.
static const auto WR0_JOINT = 16;
static const auto F1X_JOINT = 18;

class WiggleArm : public rclcpp::Node {
 public:
  WiggleArm() : Node("wiggle_arm"), wiggle_state{WIGGLE_DOWN}, initialized{false} {
    declare_parameter("command_interval_sec", 3.0);  // how frequently to send commands
    const auto command_interval_sec = std::chrono::duration<double>{get_parameter("command_interval_sec").as_double()};
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&WiggleArm::joint_states_callback, this, std::placeholders::_1));
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    timer_ = create_wall_timer(command_interval_sec, std::bind(&WiggleArm::timer_callback, this));
  }

 private:
  std_msgs::msg::Float64MultiArray command_start;
  std_msgs::msg::Float64MultiArray command_wiggle_down;
  std_msgs::msg::Float64MultiArray command_wiggle_up;
  WiggleState wiggle_state;
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
      command_wiggle_down.data.at(WR0_JOINT) += 1.0;
      command_wiggle_down.data.at(F1X_JOINT) -= 0.5;
      command_wiggle_up.data = msg.position;
      command_wiggle_up.data.at(WR0_JOINT) -= 1.0;
      command_wiggle_up.data.at(F1X_JOINT) -= 0.5;
      initialized = true;
    }
  }

  void timer_callback() {
    if (!initialized) {
      return;
    }
    switch (wiggle_state) {
      case START:
        RCLCPP_INFO_STREAM(get_logger(), "Starting Pose");
        command_pub_->publish(command_start);
        wiggle_state = WIGGLE_DOWN;
        break;
      case WIGGLE_DOWN:
        RCLCPP_INFO_STREAM(get_logger(), "Wiggle Down");
        command_pub_->publish(command_wiggle_down);
        wiggle_state = WIGGLE_MIDDLE;
        break;
      case WIGGLE_MIDDLE:
        RCLCPP_INFO_STREAM(get_logger(), "Starting Pose");
        command_pub_->publish(command_start);
        wiggle_state = WIGGLE_UP;
        break;
      case WIGGLE_UP:
        RCLCPP_INFO_STREAM(get_logger(), "Wiggle Up");
        command_pub_->publish(command_wiggle_up);
        wiggle_state = START;
        break;
      default:
        RCLCPP_INFO_STREAM(get_logger(), "Invalid Wiggle State");
        wiggle_state = START;
        break;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WiggleArm>());
  rclcpp::shutdown();
  return 0;
}
