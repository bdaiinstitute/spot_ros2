// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

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

struct ImageWithCameraInfo
{
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;
};
}

/**
 * @brief Provide a specialization of std::less for ImageSource to allow using ImageSource as a key in std::map.
 */
template <> struct std::less<spot_ros2::ImageSource>
{
  bool operator()(const spot_ros2::ImageSource& lhs, const spot_ros2::ImageSource& rhs) const
  {
    // If the name strings are different, use them for comparison.
    if (lhs.name != rhs.name)
    {
      return lhs.name < rhs.name;
    }
    
    // Compare the type enum to break the tie if the names are identical.
    return lhs.type < rhs.type;
  }
};
