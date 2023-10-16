// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/spot_image_sources.hpp>
#include <tl_expected/expected.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that publishes image data to middleware.
 */
class PublisherInterfaceBase {
 public:
  virtual ~PublisherInterfaceBase() {}

  virtual void createPublishers(const std::set<ImageSource>& image_sources) = 0;
  virtual tl::expected<void, std::string> publish(const std::map<ImageSource, ImageWithCameraInfo>& images) = 0;
};
}  // namespace spot_ros2
