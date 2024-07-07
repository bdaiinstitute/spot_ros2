// Based off: https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/wiggle_arm.py

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class WiggleArm : public rclcpp::Node {
 public:
  WiggleArm() : Node("wiggle_arm"), state{0} {
    declare_parameter("timer_rate", 5.0);
    const auto timer_rate = std::chrono::duration<double>{get_parameter("timer_rate").as_double()};
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    timer_ = create_wall_timer(timer_rate, std::bind(&WiggleArm::timer_callback, this));
    // This assumes the robot has an arm
    command_start.data = std::vector<double>(19, 0.0);
    command_wiggle1.data = std::vector<double>(19, 0.0);
    command_wiggle1.data.at(16) += 1.0;
    command_wiggle1.data.at(18) -= 0.5;
    command_wiggle2.data = std::vector<double>(19, 0.0);
    command_wiggle2.data.at(16) -= 1.0;
    command_wiggle2.data.at(18) -= 0.5;
  }

 private:
  std_msgs::msg::Float64MultiArray command_start;
  std_msgs::msg::Float64MultiArray command_wiggle1;
  std_msgs::msg::Float64MultiArray command_wiggle2;
  int state;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void timer_callback() {
    if (state == 0) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Wiggle 1");
      command_pub_->publish(command_wiggle1);
    } else if (state == 1) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Start");
      command_pub_->publish(command_start);
    } else if (state == 2) {
      RCLCPP_INFO_STREAM(get_logger(), "Command Wiggle 2");
      command_pub_->publish(command_wiggle2);
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
