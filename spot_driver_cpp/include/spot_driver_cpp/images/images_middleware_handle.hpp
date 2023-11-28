// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver_cpp/images/spot_image_publisher.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_tf_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_wall_timer_interface.hpp>
#include <set>
#include <map>
#include <string>
#include <tl_expected/expected.hpp>
#include <memory>
#include <unordered_map>

namespace spot_ros2::images {

struct ImagesMiddlewareHandle : public SpotImagePublisher::MiddlewareHandle {
  explicit ImagesMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);
  ~ImagesMiddlewareHandle() = default;

  void createPublishers(const std::set<ImageSource>& image_sources) override;
  tl::expected<void, std::string> publishImages(const std::map<ImageSource, ImageWithCameraInfo>& images) override;

  ParameterInterfaceBase* parameter_interface() override { return parameter_interface_.get(); }
  LoggerInterfaceBase* logger_interface() override { return logger_interface_.get(); }
  TfInterfaceBase* tf_interface() override { return tf_interface_.get(); }
  TimerInterfaceBase* timer_interface() override { return timer_interface_.get(); }

 private:
  /** @brief Shared instance of an rclcpp node to create publishers */
  std::shared_ptr<rclcpp::Node> node_;
  /** @brief instance of ParameterInterfaceBase to get ROS parameters*/
  std::unique_ptr<RclcppParameterInterface> parameter_interface_;
  /** @brief instance of LoggerInterfaceBase to send ROS log messages*/
  std::unique_ptr<RclcppLoggerInterface> logger_interface_;
  /** @brief instance of TfInterfaceBase to update static transforms*/
  std::unique_ptr<RclcppTfInterface> tf_interface_;
  /** @brief instance of TimerInterfaceBase to create callback timer*/
  std::unique_ptr<RclcppWallTimerInterface> timer_interface_;

  /** @brief Map between image topic names and image publishers. */
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> image_publishers_;

  /** @brief Map between camera info topic names and camera info publishers. */
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>> info_publishers_;
};
}  // namespace spot_ros2::images
