// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <spot_driver/interfaces/publisher_interface_base.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {
/**
 * @brief Implements PublisherInterfaceBase to use a rclcpp::Publisher.
 * @tparam MessageT ROS message type that specializes the publisher.
 */
template <typename MessageT>
class RclcppPublisherInterface final : public PublisherInterfaceBase<MessageT> {
 public:
  /**
   * @brief The constructor for RclcppPublisherInterface.
   * @param node Used to create a publisher.
   * @param topic The publisher will publish on this topic.
   */
  explicit RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic)
      : publisher_{node->create_publisher<MessageT>(topic, rclcpp::QoS{1})} {}

  void publish(const MessageT& message) const override { publisher_->publish(message); }

 private:
  std::shared_ptr<rclcpp::Publisher<MessageT>> publisher_;
};
}  // namespace spot_ros2
