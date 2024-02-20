// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <string>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that publishes a ROS message.
 * @tparam MessageT ROS message type that specializes the publisher.
 */
template <typename MessageT>
class PublisherInterfaceBase {
 public:
  virtual ~PublisherInterfaceBase() = default;
  virtual void publish(const MessageT& message) const = 0;
};
}  // namespace spot_ros2
