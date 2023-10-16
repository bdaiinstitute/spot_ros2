// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver_cpp/interfaces/publisher_interface_base.hpp>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Implements PublisherInterfaceBase to use rclcpp Publishers.
 */
class RclcppPublisherInterface : public PublisherInterfaceBase {
 public:
  explicit RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node);

  void createPublishers(const std::vector<ImageSource>& image_sources) override;
  tl::expected<void, std::string> publish(const std::map<ImageSource, ImageWithCameraInfo>& images) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> image_publishers_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>> info_publishers_;
};
}  // namespace spot_ros2
