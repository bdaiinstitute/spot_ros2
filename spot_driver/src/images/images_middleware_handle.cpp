// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <rclcpp/node.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/images/images_middleware_handle.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 10;

}  // namespace

namespace spot_ros2::images {

ImagesMiddlewareHandle::ImagesMiddlewareHandle(std::shared_ptr<rclcpp::Node> node)
    : node_{node},
      parameter_interface_{std::make_unique<RclcppParameterInterface>(node)},
      logger_interface_{std::make_unique<RclcppLoggerInterface>(node->get_logger())},
      tf_interface_{std::make_unique<RclcppTfInterface>(node)},
      timer_interface_{std::make_unique<RclcppWallTimerInterface>(node)} {}

ImagesMiddlewareHandle::ImagesMiddlewareHandle(const rclcpp::NodeOptions& node_options)
    : ImagesMiddlewareHandle(std::make_shared<rclcpp::Node>("image_publisher", node_options)) {}

void ImagesMiddlewareHandle::createPublishers(const std::set<ImageSource>& image_sources,
                                              const bool do_decompress_images) {
  image_publishers_.clear();
  info_publishers_.clear();

  for (const auto& image_source : image_sources) {
    // Since these topic names do not have a leading `/` character, they will be published within the namespace of the
    // node, which should match the name of the robot. For example, the topic for the front left RGB camera will
    // ultimately appear as `/MyRobotName/camera/frontleft/image`.
    const auto image_topic_name = toRosTopic(image_source);

    if ((image_source.type != SpotImageType::RGB) || do_decompress_images) {
      image_publishers_.try_emplace(
          image_topic_name, node_->create_publisher<sensor_msgs::msg::Image>(
                                image_topic_name + "/image", rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth))));
    } else {
      compressed_image_publishers_.try_emplace(
          image_topic_name,
          node_->create_publisher<sensor_msgs::msg::CompressedImage>(
              image_topic_name + "/image/compressed", rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth))));
    }

    info_publishers_.try_emplace(image_topic_name, node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                                                       image_topic_name + "/camera_info",
                                                       rclcpp::QoS(rclcpp::KeepLast(kPublisherHistoryDepth))));
  }
}

void ImagesMiddlewareHandle::tryPublishImage(std::string const& image_topic_name,
                                             sensor_msgs::msg::Image const& image) {
  image_publishers_.at(image_topic_name)->publish(image);
}

void ImagesMiddlewareHandle::tryPublishImage(std::string const& image_topic_name,
                                             sensor_msgs::msg::CompressedImage const& compressed_image) {
  compressed_image_publishers_.at(image_topic_name)->publish(compressed_image);
}

void ImagesMiddlewareHandle::tryPublishImageInfo(std::string const& image_topic_name,
                                                 sensor_msgs::msg::CameraInfo const& camera_info) {
  info_publishers_.at(image_topic_name)->publish(camera_info);
}

}  // namespace spot_ros2::images
