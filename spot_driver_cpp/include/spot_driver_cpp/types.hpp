// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <functional>
#include <string>

namespace spot_ros2 {
/** @brief Represents the six different cameras on Spot. */
enum class SpotCamera {
  BACK,
  FRONTLEFT,
  FRONTRIGHT,
  LEFT,
  RIGHT,
  HAND,
};

/** @brief Represents the three types of images Spot can capture. */
enum class SpotImageType {
  RGB,
  DEPTH,
  DEPTH_REGISTERED,
};

/** @brief Defines the name and type of an image source. */
struct ImageSource {
  /** @brief Name of the image source. */
  SpotCamera camera;

  /** @brief Type of the image source. */
  SpotImageType type;

  /** @brief Allows comparing one ImageSource instance with another. */
  bool operator==(const ImageSource& e) const { return e.camera == camera && e.type == type; }
};

/** @brief Stores an Image message and a corresponding CameraInfo message together. */
struct ImageWithCameraInfo {
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;
};
}  // namespace spot_ros2

/**
 * @brief Provide a specialization of std::less for ImageSource to allow using ImageSource as a key in std::map.
 */
template <>
struct std::less<spot_ros2::ImageSource> {
  bool operator()(const spot_ros2::ImageSource& lhs, const spot_ros2::ImageSource& rhs) const {
    // If the camera IDs are different, use them for comparison.
    if (lhs.camera != rhs.camera) {
      return lhs.camera < rhs.camera;
    }

    // Compare the type enum to break the tie if the names are identical.
    return lhs.type < rhs.type;
  }
};
