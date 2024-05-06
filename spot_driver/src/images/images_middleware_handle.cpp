// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/node.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/images/images_middleware_handle.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 10;

}  // namespace

namespace spot_ros2::images {

ImagesMiddlewareHandle::ImagesMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node) : node_{node} {}

ImagesMiddlewareHandle::ImagesMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : ImagesMiddlewareHandle(std::make_shared<rclcpp::Node>("image_publisher", node_options)) {}

void ImagesMiddlewareHandle::createPublishers(const std::set<ImageSource>& image_sources, bool uncompress_images,
                                              bool publish_compressed_images) {
  image_publishers_.clear();
  info_publishers_.clear();

  for (const auto& image_source : image_sources) {
    // Since these topic names do not have a leading `/` character, they will be published within the namespace of the
    // node, which should match the name of the robot. For example, the topic for the front left RGB camera will
    // ultimately appear as `/MyRobotName/camera/frontleft/image`.
    const auto image_topic_name = toRosTopic(image_source);

    if (image_source.type == SpotImageType::RGB && publish_compressed_images) {
      compressed_image_publishers_.try_emplace(
          image_topic_name, node_->create_publisher<sensor_msgs::msg::CompressedImage>(
                                image_topic_name + "/compressed", makePublisherQoS(kPublisherHistoryDepth)));
    }
    if (uncompress_images || (image_source.type != SpotImageType::RGB)) {
      image_publishers_.try_emplace(
          image_topic_name, node_->create_publisher<sensor_msgs::msg::Image>(image_topic_name + "/image",
                                                                             makePublisherQoS(kPublisherHistoryDepth)));
    }
    info_publishers_.try_emplace(image_topic_name,
                                 node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                                     image_topic_name + "/camera_info", makePublisherQoS(kPublisherHistoryDepth)));
  }
}

tl::expected<void, std::string> ImagesMiddlewareHandle::publishImages(
    const std::map<ImageSource, ImageWithCameraInfo>& images,
    const std::map<ImageSource, CompressedImageWithCameraInfo>& compressed_images) {
  std::set<std::string> camera_infos_sent;
  for (const auto& [image_source, image_data] : images) {
    const auto image_topic_name = toRosTopic(image_source);
    try {
      image_publishers_.at(image_topic_name)->publish(image_data.image);
    } catch (const std::out_of_range& e) {
      return tl::make_unexpected("No image publisher exists for image topic `" + image_topic_name + "`.");
    }
    try {
      info_publishers_.at(image_topic_name)->publish(image_data.info);
      camera_infos_sent.insert(image_topic_name);
    } catch (const std::out_of_range& e) {
      return tl::make_unexpected("No camera_info publisher exists for camera info topic`" + image_topic_name + "`.");
    }
  }
  for (const auto& [image_source, compressed_image_data] : compressed_images) {
    const auto image_topic_name = toRosTopic(image_source);
    try {
      compressed_image_publishers_.at(image_topic_name)->publish(compressed_image_data.image);
    } catch (const std::out_of_range& e) {
      return tl::make_unexpected("No compressed image publisher exists for image topic `" + image_topic_name + "`.");
    }
    auto camera_info_insert_result = camera_infos_sent.insert(image_topic_name);
    if (camera_info_insert_result.second) {
      try {
        info_publishers_.at(image_topic_name)->publish(compressed_image_data.info);
      } catch (const std::out_of_range& e) {
        return tl::make_unexpected("No camera_info publisher exists for camera info topic`" + image_topic_name + "`.");
      }
    }
  }
  return {};
}

}  // namespace spot_ros2::images
