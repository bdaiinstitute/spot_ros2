// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <set>
#include <spot_driver/images/spot_image_publisher.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <string>
#include <tl_expected/expected.hpp>
#include <unordered_map>

namespace spot_ros2::images {
/**
 * @brief Implementation of SpotImagePublisher::MiddlewareHandle
 */
class ImagesMiddlewareHandle : public SpotImagePublisher::MiddlewareHandle {
 public:
  /**
   * @brief Constructor for ImagesMiddlewareHandle
   *
   * @param node  A shared_ptr to an instance of a rclcpp::Node
   */
  explicit ImagesMiddlewareHandle(std::shared_ptr<rclcpp::Node> node);

  /**
   * @brief Constructor for ImagesMiddlewareHandle which creates an instance of an rclcpp::node
   *
   * @param node_options Options for rclcpp::node initialization
   */
  explicit ImagesMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  ~ImagesMiddlewareHandle() = default;

  /**
   * @brief Populates the image_publishgers_ and info_publishers_ members with image and camera info publishers.
   * @param image_sources Set of ImageSources. A publisher will be created for each ImageSource.
   */
  void createPublishers(const std::set<ImageSource>& image_sources) override;

  /**
   * @brief Publishes image and camera info messages to ROS 2 topics.
   * @param images Map of image sources to image and camera info data.
   * @return If all images were published successfully, returns void. If there was an error, returns an error message.
   */
  tl::expected<void, std::string> publishImages(const std::map<ImageSource, ImageWithCameraInfo>& images) override;

  ParameterInterfaceBase* parameter_interface() override { return parameter_interface_.get(); }
  LoggerInterfaceBase* logger_interface() override { return logger_interface_.get(); }
  TfInterfaceBase* tf_interface() override { return tf_interface_.get(); }
  TimerInterfaceBase* timer_interface() override { return timer_interface_.get(); }

  std::shared_ptr<rclcpp::Node> node() override { return node_; }

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
