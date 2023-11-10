// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
#include <tl_expected/expected.hpp>

#include <map>
#include <string>
#include <vector>

namespace spot_ros2 {
struct GetImagesResult {
  std::map<ImageSource, ImageWithCameraInfo> images_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};

/**
 * @brief Defines an interface for a class to connect to and interact with Spot.
 */
class ImageApi {
 public:
  virtual ~ImageApi() {}
  virtual tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) = 0;
  virtual tl::expected<builtin_interfaces::msg::Time, std::string> convertRobotTimeToLocalTime(
      const google::protobuf::Timestamp& robot_timestamp) = 0;
};
}  // namespace spot_ros2
