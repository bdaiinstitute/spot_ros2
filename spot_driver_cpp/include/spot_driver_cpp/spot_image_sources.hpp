// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/image/image_client.h>
#include <tl_expected/expected.hpp>
#include <string>
#include <map>

namespace spot_ros2
{
enum class SpotImageType
{
  RGB,
  DEPTH,
  DEPTH_REGISTERED,
};

struct ImageSource
{
  std::string name;
  SpotImageType type;
};

std::string toRosTopic(const ImageSource& image_source);

std::string toSpotImageSourceName(const ImageSource& image_source);

tl::expected<ImageSource, std::string> fromSpotImageSourceName(const std::string& source_name);

/**
 * @brief Create a list of image sources corresponding to the specified image types.
 * 
 * @param get_rgb_images Sets whether to request RGB images.
 * @param get_depth_images Sets whether to request depth images.
 * @param get_depth_registered_images Sets whether to request registered depth images.
 * @param has_hand_camera Sets whether to request images from the hand camera.
 * @return ImageSources 
 */
std::vector<ImageSource> createImageSourcesList(const bool get_rgb_images, const bool get_depth_images, const bool get_depth_registered_images, const bool has_hand_camera);
}

namespace std
{
  template <> struct less<spot_ros2::ImageSource>
  {
    bool operator()(const spot_ros2::ImageSource& lhs, const spot_ros2::ImageSource& rhs) const
    {
      if (lhs.name != rhs.name)
      {
        return lhs.name < rhs.name;
      }

      return lhs.type < rhs.type;
    }
  };
}
