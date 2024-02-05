// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/image/image_client.h>
#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Create the ROS topic name corresponding to an ImageSource.
 *
 * @param image_source Input image source.
 * @return ROS topic name for the input image source.
 */
std::string toRosTopic(const ImageSource& image_source);

/**
 * @brief Create the Spot SDK source name corresponding to an ImageSource.
 *
 * @param image_source Input image source.
 * @return Spot SDK source name for the input image source.
 */
std::string toSpotImageSourceName(const ImageSource& image_source);

/**
 * @brief Create an ImageSource corresponding to a Spot SDK source name.
 *
 * @param source_name Input source name.
 * @return If the input source name was successfully parsed, return an ImageSource.
 * @return If the input source name does not match the expected name of any known Spot SDK source, return an error.
 */
tl::expected<ImageSource, std::string> fromSpotImageSourceName(const std::string& source_name);

/**
 * @brief Create a set of image sources corresponding to the specified image types.
 * @details We represent the collection of ImageSources as a std::set to clearly communicate the requirement that we
 * must only send one request to Spot for a given combination of camera type and image type.
 *
 * @param get_rgb_images Sets whether to request RGB images.
 * @param get_depth_images Sets whether to request depth images.
 * @param get_depth_registered_images Sets whether to request registered depth images.
 * @param has_hand_camera Sets whether to request images from the hand camera.
 * @return A set of ImageSources which represents all requested image and camera types.
 */
std::set<ImageSource> createImageSources(const bool get_rgb_images, const bool get_depth_images,
                                         const bool get_depth_registered_images, const bool has_hand_camera);
}  // namespace spot_ros2
