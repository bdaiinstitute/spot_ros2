// Based off: https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/joint_control/wiggle_arm.py

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum WiggleState { WIGGLE_DOWN, WIGGLE_MIDDLE, WIGGLE_UP, RESET };

class WiggleArm : public rclcpp::Node {
 public:
  WiggleArm() : Node("wiggle_arm"), wiggle_state{WIGGLE_DOWN}, initialized{false} {
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

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&WiggleArm::joint_states_callback, this, std::placeholders::_1));
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
    timer_ = create_wall_timer((std::chrono::milliseconds)(static_cast<int>(1000. / command_rate)),
                               std::bind(&WiggleArm::timer_callback, this));
  }

 private:
  std::vector<double> nominal_joint_angles_;
  std::vector<int64_t> joints_to_wiggle_;
  std::vector<double> wiggle_up_offsets_;
  std::vector<double> wiggle_down_offsets_;
  size_t njoints_to_wiggle_;
  // Command to send to the robot
  std_msgs::msg::Float64MultiArray command_;
  // Parameters
  WiggleState wiggle_state;
  int points_per_motion_;
  bool initialized;
  int count_;
  // Timer, publishers, subscribers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    // Save the starting joint angles
    if (!initialized) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      nominal_joint_angles_ = msg.position;
      command_.data = msg.position;
      initialized = true;
    }
  }

  void timer_callback() {
    // Only send a command if we have initialized with the starting joint angles
    if (!initialized) {
      return;
    }
    // Check if we need to switch state
    if (count_ > points_per_motion_) {
      RCLCPP_INFO_STREAM(get_logger(), "Reset");
      switch (wiggle_state) {
        case WIGGLE_DOWN:
          RCLCPP_INFO_STREAM(get_logger(), "Middle");
          wiggle_state = WIGGLE_MIDDLE;
          break;
        case WIGGLE_MIDDLE:
          RCLCPP_INFO_STREAM(get_logger(), "Up");
          wiggle_state = WIGGLE_UP;
          break;
        case WIGGLE_UP:
          RCLCPP_INFO_STREAM(get_logger(), "Reset");
          wiggle_state = RESET;
          break;
        case RESET:
          RCLCPP_INFO_STREAM(get_logger(), "Down");
          wiggle_state = WIGGLE_DOWN;
          break;
      }
      count_ = 0;
    }
    // Percentage we are through the desired motion
    const double percentage = static_cast<float>(count_) / points_per_motion_;
    // Fill in the command with the appropriate joint angles given the state
    switch (wiggle_state) {
      case WIGGLE_DOWN:
        for (size_t i = 0; i < njoints_to_wiggle_; i++) {
          const auto joint = joints_to_wiggle_.at(i);
          command_.data.at(joint) = percentage * wiggle_down_offsets_.at(i) + nominal_joint_angles_.at(joint);
        }
        break;
      case WIGGLE_MIDDLE:
        for (size_t i = 0; i < njoints_to_wiggle_; i++) {
          const auto joint = joints_to_wiggle_.at(i);
          command_.data.at(joint) = (1 - percentage) * wiggle_down_offsets_.at(i) + nominal_joint_angles_.at(joint);
        }
        break;
      case WIGGLE_UP:
        for (size_t i = 0; i < njoints_to_wiggle_; i++) {
          const auto joint = joints_to_wiggle_.at(i);
          command_.data.at(joint) = percentage * wiggle_up_offsets_.at(i) + nominal_joint_angles_.at(joint);
        }
        break;
      case RESET:
        for (size_t i = 0; i < njoints_to_wiggle_; i++) {
          const auto joint = joints_to_wiggle_.at(i);
          command_.data.at(joint) = (1 - percentage) * wiggle_up_offsets_.at(i) + nominal_joint_angles_.at(joint);
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
  rclcpp::spin(std::make_shared<WiggleArm>());
  rclcpp::shutdown();
  return 0;
}
