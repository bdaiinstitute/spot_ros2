// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <future>
#include <memory>
#include <mutex>
#include <spot_driver/interfaces/urdf_robot_model_interface.hpp>

#include <rclcpp/node.hpp>
#include <set>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace {
constexpr auto kRobotDescriptionTopic = "robot_description";
}

namespace spot_ros2 {

UrdfRobotModelInterface::UrdfRobotModelInterface(const std::shared_ptr<rclcpp::Node>& node)
    : has_robot_model_{false},
      subscriber_callback_group_{node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)},
      urdf_subscriber_{node->create_subscription<std_msgs::msg::String>(
          kRobotDescriptionTopic, rclcpp::QoS(1).reliable().transient_local(),
          [this](const std::shared_ptr<std_msgs::msg::String> msg) {
            onRobotDescription(*msg);
          })}

{}

tl::expected<std::set<std::string>, std::string> UrdfRobotModelInterface::getFrameIds() const {
  if (!has_robot_model_) {
    return tl::make_unexpected("no robot model yet");
  }

  std::lock_guard lock{robot_model_mutex_};

  std::set<std::string> out;
  for (const auto& link : model_->links_) {
    out.insert(link.first);
  }
  return out;
}

void UrdfRobotModelInterface::onRobotDescription(const std_msgs::msg::String& msg) {
  std::lock_guard lock{robot_model_mutex_};
  model_ = urdf::parseURDF(msg.data);
  urdf_subscriber_.reset();
  has_robot_model_ = true;
}

}  // namespace spot_ros2
