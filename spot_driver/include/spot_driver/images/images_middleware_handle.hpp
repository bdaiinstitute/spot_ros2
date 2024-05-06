// Copyright (c) 2023-2024 The AI Institute LLC. All rights reserved.

#pragma once

#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <set>
#include <spot_driver/images/spot_image_publisher.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
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
  explicit ImagesMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Constructor for ImagesMiddlewareHandle which creates an instance of an rclcpp::node
   *
   * @param node_options Options for rclcpp::node initialization
   */
  explicit ImagesMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Populates the image_publishgers_ and info_publishers_ members with image and camera info publishers.
   * @param image_sources Set of ImageSources. A publisher will be created for each ImageSource.
   */
  void createPublishers(const std::set<ImageSource>& image_sources, bool uncompress_images,
                        bool publish_compressed_images) override;

  /**
   * @brief Publishes (compressed) images and camera info messages to ROS 2 topics.
   * @param images Map of image sources to image and camera info data.
   * @param compressed_images Map of image sources to compressed image and camera info data.
   * @return If all images were published successfully, returns void. If there was an error, returns an error message.
   */
  tl::expected<void, std::string> publishImages(
      const std::map<ImageSource, ImageWithCameraInfo>& images,
      const std::map<ImageSource, CompressedImageWithCameraInfo>& compressed_images) override;

 private:
  /** @brief Shared instance of an rclcpp node to create publishers */
  std::shared_ptr<rclcpp::Node> node_;

  /** @brief Map between image topic names and image publishers. */
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> image_publishers_;

  /** @brief Map between image topic names and compressed image publishers. */
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>>
      compressed_image_publishers_;

  /** @brief Map between camera info topic names and camera info publishers. */
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>> info_publishers_;
};
}  // namespace spot_ros2::images
