// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>
#include <spot_driver/api/middleware_handle_base.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/interfaces/image_client_interface.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/types.hpp>
#include <string>

namespace spot_ros2::images {
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
   * @brief A handle class around rclcpp::Node operations for SpotImagePublisher
   */
  class MiddlewareHandle : public MiddlewareHandleBase {
   public:
    virtual ~MiddlewareHandle() = default;

    virtual void createPublishers(const std::set<ImageSource>& image_sources, bool uncompress_images,
                                  bool publish_compressed_images) = 0;
    virtual tl::expected<void, std::string> publishImages(
        const std::map<ImageSource, ImageWithCameraInfo>& images,
        const std::map<ImageSource, CompressedImageWithCameraInfo>& compressed_images) = 0;
  };

  /**
   * @brief Constructor for SpotImagePublisher, which allows setting the specific implementation of the interface
   * classes which are used by the node.
   * @details This can be in production and to perform dependency injection in unit tests
   *
   * @param image_client_interface  A shared_ptr to an instance of a class that implements ImageClientInterface.
   * @param middleware_handle A unique_ptr to an instance of a class that implements
   * SpotImagePublisher::MiddlewareHandle
   * @param has_arm A flag indicating if the Spot in use has an arm. Needed to generate image sources during
   * initialization.
   */
  SpotImagePublisher(const std::shared_ptr<ImageClientInterface>& image_client_interface,
                     std::unique_ptr<MiddlewareHandle> middleware_handle,
                     std::unique_ptr<ParameterInterfaceBase> parameters, std::unique_ptr<LoggerInterfaceBase> logger,
                     std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                     std::unique_ptr<TimerInterfaceBase> timer, bool has_arm = false);

  /**
   * @brief Connect to Spot and start publishing image data.
   *
   * @return True, if connection and authentication succeeded.
   * @return False, if a required user-defined parameter was not set, if the attempt to connect to Spot fails, or if the
   * attempt to authenticate with Spot using the provided credentials fails.
   */
  [[nodiscard]] bool initialize();

 private:
  /**
   * @brief Callback function which is called through timer_interface_.
   * @details Requests image data from Spot, and then publishes the images and static camera transforms.
   */
  void timerCallback(bool uncompress_images, bool publish_compressed_images);

  /**
   * @brief Image request message which is set when SpotImagePublisher::initialize() is called.
   * @details This is generated only once and then cached because the configuration of which cameras to request images
   * from, what quality to use, etc., does not dynamically change while the driver is running.
   */
  std::optional<::bosdyn::api::GetImageRequest> image_request_message_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<ImageClientInterface> image_client_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;

  std::unique_ptr<ParameterInterfaceBase> parameters_;
  std::unique_ptr<LoggerInterfaceBase> logger_;
  std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_;
  std::unique_ptr<TimerInterfaceBase> timer_;

  bool has_arm_;
};
}  // namespace spot_ros2::images
