// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/qos.hpp>

namespace spot_ros2 {

/**
 * @brief Base class for common ROS2 publisher definitions
 */
class MiddlewareHandleBase {
 public:
  rclcpp::QoS makePublisherQoS(size_t const publisherHistoryDepth) const {
    // most compatible publisher durability: transient local
    // most compatible publisher reliabilty: reliable
    // see also
    // https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-compatibilities
    return rclcpp::QoS(publisherHistoryDepth).transient_local().reliable();
  }
};
}  // namespace spot_ros2
