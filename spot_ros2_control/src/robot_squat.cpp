#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RobotSquat : public rclcpp::Node {
 public:
  RobotSquat() : Node("robot_squat"), standing{true} {
    declare_parameter("timer_rate", 5.0);
    const auto timer_rate = std::chrono::duration<double>{get_parameter("timer_rate").as_double()};
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    timer_ = create_wall_timer(timer_rate, std::bind(&RobotSquat::timer_callback, this));
    // This assumes the robot has an arm
    command_stand.data = std::vector<double>(12, 0.0);
    command_squat.data = {0.15, 1.3, -2.25, -0.15, 1.3, -2.25, 0.15, 1.3, -2.25, -0.15, 1.3, -2.25};
  }

 private:
  std_msgs::msg::Float64MultiArray command_squat;
  std_msgs::msg::Float64MultiArray command_stand;
  bool standing;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void timer_callback() {
    if (standing) {
      RCLCPP_INFO_STREAM(get_logger(), "Command squat");
      command_pub_->publish(command_squat);
      standing = false;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Command stand");
      command_pub_->publish(command_stand);
      standing = true;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSquat>());
  rclcpp::shutdown();
  return 0;
}
