// based off: https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/noarm_squat.py

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class NoarmSquat : public rclcpp::Node {
 public:
  NoarmSquat() : Node("noarm_squat"), standing{true}, initialized{false} {
    declare_parameter("stand_joint_angles", std::vector<double>{});
    declare_parameter("squat_joint_angles", std::vector<double>{});
    declare_parameter("command_rate", 50.0);       // how frequently to send commands in Hz
    declare_parameter("seconds_per_motion", 5.0);  // how many seconds the squat and stand motions should take

    command_stand_.data = get_parameter("stand_joint_angles").as_double_array();
    command_squat_.data = get_parameter("squat_joint_angles").as_double_array();
    command_rate_ = get_parameter("command_rate").as_double();
    seconds_per_motion_ = get_parameter("seconds_per_motion").as_double();

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&NoarmSquat::joint_states_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer((std::chrono::milliseconds)((int)(1000. / command_rate_)),
                               std::bind(&NoarmSquat::timer_callback, this));
  }

 private:
  std::vector<std_msgs::msg::Float64MultiArray> standup_commands_;
  std_msgs::msg::Float64MultiArray command_squat_;
  std_msgs::msg::Float64MultiArray command_stand_;
  std::vector<double> initial_joint_positions_;
  int command_rate_;
  double seconds_per_motion_;
  bool standing;
  bool initialized;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    if (!initialized) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      initial_joint_positions_ = msg.position;
      initialized = true;
    }
  }

  void get_standup_commands() {}

  void timer_callback() {
    if (!initialized) {
      return;
    }
    if (standing) {
      RCLCPP_INFO_STREAM(get_logger(), "Squat");
      command_pub_->publish(command_squat_);
      standing = false;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "Stand");
      command_pub_->publish(command_stand_);
      standing = true;
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoarmSquat>());
  rclcpp::shutdown();
  return 0;
}
