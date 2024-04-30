// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>

#include <map>
#include <string>
#include <vector>

namespace spot_ros2 {

struct GetImagesResult {
  std::map<ImageSource, ImageWithCameraInfo> images_;
  std::map<ImageSource, CompressedImageWithCameraInfo> compressed_images_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};

/**
 * @brief Defines an interface for a class to connect to and interact with Spot's Image client.
 */
class ImageClientInterface {
 public:
  // ImageClientInterface is move-only
  ImageClientInterface() = default;
  ImageClientInterface(ImageClientInterface&& other) = default;
  ImageClientInterface(const ImageClientInterface&) = delete;
  ImageClientInterface& operator=(ImageClientInterface&& other) = default;
  ImageClientInterface& operator=(const ImageClientInterface&) = delete;
  virtual ~ImageClientInterface() = default;
  virtual tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request,
                                                               bool uncompress_images,
                                                               bool publish_compressed_images) = 0;
};
}  // namespace spot_ros2
