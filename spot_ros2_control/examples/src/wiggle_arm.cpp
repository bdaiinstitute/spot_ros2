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
  WiggleArm() : Node("wiggle_arm"), wiggle_state{START}, initialized{false} {
    declare_parameter("joints_to_wiggle", std::vector<int>{});
    declare_parameter("wiggle_up_offsets", std::vector<double>{});
    declare_parameter("wiggle_down_offsets", std::vector<double>{});
    declare_parameter("command_rate", 50.0);       // how frequently to send commands in Hz
    declare_parameter("seconds_per_motion", 2.0);  // how many seconds the squat and stand motions should take

    const auto command_rate = get_parameter("command_rate").as_double();
    const auto seconds_per_motion = get_parameter("seconds_per_motion").as_double();
    points_per_motion_ = static_cast<int>(command_rate * seconds_per_motion);

    joints_to_wiggle_ = get_parameter("joints_to_wiggle").as_integer_array();
    wiggle_up_offsets_ = get_parameter("wiggle_up_offsets").as_double_array();
    wiggle_down_offsets_ = get_parameter("wiggle_down_offsets").as_double_array();

    njoints_to_wiggle_ = joints_to_wiggle_.size();

    std::cout << "NJOINTS TO WIGGLE " << njoints_to_wiggle_ << std::endl;

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&WiggleArm::joint_states_callback, this, std::placeholders::_1));
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    timer_ = create_wall_timer((std::chrono::milliseconds)(static_cast<int>(1000. / command_rate)),
                               std::bind(&WiggleArm::timer_callback, this));
  }

 private:
  std::vector<double> nominal_joint_angles_;
  std::vector<long> joints_to_wiggle_;
  std::vector<double> wiggle_up_offsets_;
  std::vector<double> wiggle_down_offsets_;

  std::vector<double> diff_up_;
  std::vector<double> diff_down_;

  // Command to send to the robot
  std_msgs::msg::Float64MultiArray command_;

  WiggleState wiggle_state;
  int points_per_motion_;
  bool initialized;
  int count_;
  size_t njoints_to_wiggle_;
  // Timer, publishers, subscribers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    if (!initialized) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      nominal_joint_angles_ = msg.position;
      command_.data = msg.position;
      for (size_t i=0; i<njoints_to_wiggle_; i++){
        diff_up_.push_back(wiggle_up_offsets_.at(i) - nominal_joint_angles_.at(i));
        diff_down_.push_back(wiggle_down_offsets_.at(i) - nominal_joint_angles_.at(i));
      }
      initialized = true;
    }
  }

  void timer_callback() {
    if (!initialized) {
      return;
    }
    if (count_ > points_per_motion_) {
      RCLCPP_INFO_STREAM(get_logger(), "Reset");
      switch (wiggle_state) {
        case START:
          wiggle_state = WIGGLE_DOWN;
          break;
        case WIGGLE_DOWN:
          wiggle_state = WIGGLE_MIDDLE;
          break;
        case WIGGLE_MIDDLE:
          wiggle_state = WIGGLE_UP;
          break;
        case WIGGLE_UP:
          wiggle_state = START;
          break;
      }
      count_ = 0;
    }
    const double percentage = count_ / static_cast<float>(points_per_motion_);
    switch (wiggle_state) {
      case START:
        for (size_t i = 0; i < njoints_to_wiggle_; i++) {
          const auto joint = joints_to_wiggle_.at(i);
          command_.data.at(joint) = percentage * diff_up_.at(i) + nominal_joint_angles_.at(joint);
        }
        break;
      case WIGGLE_DOWN:

        break;
      case WIGGLE_MIDDLE:

        break;
      case WIGGLE_UP:

        break;
    }
    command_pub_->publish(command_);
    count_++;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WiggleArm>());
  rclcpp::shutdown();
  return 0;
}
