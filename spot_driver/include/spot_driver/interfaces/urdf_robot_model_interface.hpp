// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/interfaces/robot_model_interface_base.hpp>

#include <set>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for classes that parse data from the robot model.
 */
class UrdfRobotModelInterface final : public RobotModelInterfaceBase {
 public:
  UrdfRobotModelInterface();

  [[nodiscard]] tl::expected<std::set<std::string>, std::string> getFrameIds() const override;
};
}  // namespace spot_ros2
