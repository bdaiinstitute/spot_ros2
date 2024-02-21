// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <set>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for classes that parse data from the robot model.
 */
class RobotModelInterfaceBase {
 public:
  virtual ~RobotModelInterfaceBase() = default;

  [[nodiscard]] virtual tl::expected<std::set<std::string>, std::string> getFrameIds() const = 0;
};
}  // namespace spot_ros2
