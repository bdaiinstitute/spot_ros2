// based off: https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/noarm_squat.py

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum SquatState { INITIALIZING, SQUATTING, STANDING };
class NoarmSquat : public rclcpp::Node {
 public:
  NoarmSquat() : Node("noarm_squat"), squat_state{INITIALIZING}, initialized_{false}, count_{0}, njoints_{12} {
    declare_parameter("stand_joint_angles", std::vector<double>{0.12, 0.72, -1.45, -0.12, 0.72, -1.45, 0.12, 0.72,
                                                                -1.45, -0.12, 0.72, -1.45});
    declare_parameter("squat_joint_angles", std::vector<double>{0.15, 1.30, -2.25, -0.15, 1.30, -2.25, 0.15, 1.30,
                                                                -2.25, -0.15, 1.30, -2.25});
    declare_parameter("command_rate", 50.0);       // how frequently to send commands in Hz
    declare_parameter("seconds_per_motion", 5.0);  // how many seconds the squat and stand motions should take

    stand_joint_angles_ = get_parameter("stand_joint_angles").as_double_array();
    squat_joint_angles_ = get_parameter("squat_joint_angles").as_double_array();

    for (int i = 0; i < njoints_; i++) {
      diff_squat_stand_.push_back(squat_joint_angles_.at(i) - stand_joint_angles_.at(i));
    }

    command_rate_ = get_parameter("command_rate").as_double();
    seconds_per_motion_ = get_parameter("seconds_per_motion").as_double();

    points_per_motion_ = static_cast<int>(command_rate_ * seconds_per_motion_);
    command_.data = std::vector<double>(njoints_, 0.0);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&NoarmSquat::joint_states_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer((std::chrono::milliseconds)(static_cast<int>(1000. / command_rate_)),
                               std::bind(&NoarmSquat::timer_callback, this));
  }

 private:
  std::vector<double> stand_joint_angles_;
  std::vector<double> squat_joint_angles_;
  std::vector<double> diff_squat_stand_;
  std::vector<double> diff_init_squat_;
  std_msgs::msg::Float64MultiArray command_;
  std::vector<double> initial_joint_positions_;
  SquatState squat_state;
  int command_rate_;
  double seconds_per_motion_;
  int points_per_motion_;
  bool initialized_;
  int count_;
  int njoints_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    if (!initialized_) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      initial_joint_positions_ = msg.position;
      initialized_ = true;
    }
  }

  void timer_callback() {
    if (!initialized_) {
      return;
    }
    if (count_ > points_per_motion_) {
      RCLCPP_INFO_STREAM(get_logger(), "Reset");
      count_ = 0;
    }
    const double percentage = count_ / static_cast<float>(points_per_motion_);
    for (int i = 0; i < 12; i++) {
      command_.data.at(i) = percentage * diff_squat_stand_.at(i) + stand_joint_angles_.at(i);
    }
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
