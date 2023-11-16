// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/middleware_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>
#include <spot_driver_cpp/interfaces/publisher_interface_base.hpp>
#include <spot_driver_cpp/interfaces/tf_interface_base.hpp>
#include <spot_driver_cpp/interfaces/timer_interface_base.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/types.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Create a Spot API GetImageRequest message to request images from the specified sources using the specified
 * options.
 * @details Requests for depth images will always use FORMAT_RAW at quality 100.0.
 *
 * @param sources Set of image sources. Defines which cameras to request images from.
 * @param has_rgb_cameras Set this to true if Spot's 2D body cameras can capture RGB image data.
 * @param rgb_image_quality A value in the range from 0.0 to 100.0 which defines the level of compression to use when
 * requesting JPEG-compressed RGB image data.
 * @param get_raw_rgb_images If true, request raw images from Spot's 2D body cameras. If false, request JPEG-compressed
 * images.
 * @return A GetImageRequest message equivalent to the input parameters.
 */
::bosdyn::api::GetImageRequest createImageRequest(const std::set<ImageSource>& sources, const bool has_rgb_cameras,
                                                  const double rgb_image_quality, const bool get_raw_rgb_images);

/**
 * @brief A class to connect to and authenticate with Spot, retrieve images from its cameras, and publish the images to
 * the middleware.
 */
class SpotImagePublisher {
 public:
  /**
   * @brief Constructor for SpotImagePublisher, which allows setting the specific implementation of the interface
   * classes which are used by the node.
   * @details This is used to perform dependency injection in unit tests.
   *
   * @param image_client_api  A unique_ptr to an instance of a class that implements ImageClientApi.
   * @param timer_interface A unique_ptr to an instance of a class that implements TimerInterfaceBase.
   * @param publisher_interface  A unique_ptr to an instance of a class that implements PublisherInterfaceBase.
   * @param parameter_interface  A unique_ptr to an instance of a class that implements ParameterInterfaceBase.
   * @param tf_interface  A unique_ptr to an instance of a class that implements TfInterfaceBase.
   */
  SpotImagePublisher(std::unique_ptr<ImageClientApi> image_client_api,
                     std::shared_ptr<MiddlewareInterface> middleware_interface, bool has_arm = false);

  /**
   * @brief Constuctor for SpotImagePublisher, which creates instances of rclcpp-specialized interface classes
   * using the provided node.
   *
   * @param node ROS 2 node to use when creating the interfaces.
   * @param image_client_api Spot SDK service client that can retrieve image data from spot cameras
   * @param has_arm get hand camera data if arm is a part of the current configuration
   */
  explicit SpotImagePublisher(const std::shared_ptr<rclcpp::Node>& node,
                              std::unique_ptr<ImageClientApi> image_client_api, bool has_arm = false);

  /**
   * @brief Connect to Spot and start publishing image data.
   *
   * @return True, if connection and authentication succeeded.
   * @return False, if a required user-defined parameter was not set, if the attempt to connect to Spot fails, or if the
   * attempt to authenticate with Spot using the provided credentials fails.
   */
  bool initialize();

 private:
  /**
   * @brief Callback function which is called through timer_interface_.
   * @details Requests image data from Spot, and then publishes the images and static camera transforms.
   */
  void timerCallback();

  /**
   * @brief Image request message which is set when SpotImagePublisher::initialize() is called.
   * @details This is generated only once and then cached because the configuration of which cameras to request images
   * from, what quality to use, etc., does not dynamically change while the driver is running.
   */
  std::optional<::bosdyn::api::GetImageRequest> image_request_message_;

  // Interface classes to interact with Spot and the middleware.
  std::unique_ptr<ImageClientApi> image_client_api_;
  std::shared_ptr<MiddlewareInterface> middleware_interface_;
  bool has_arm_;
};
}  // namespace spot_ros2
