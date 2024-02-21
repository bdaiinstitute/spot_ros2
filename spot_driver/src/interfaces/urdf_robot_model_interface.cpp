// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/interfaces/urdf_robot_model_interface.hpp>

#include <set>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

UrdfRobotModelInterface::UrdfRobotModelInterface() {}

tl::expected<std::set<std::string>, std::string> UrdfRobotModelInterface::getFrameIds() const {
  return tl::make_unexpected("Not yet implemented.");
}
}  // namespace spot_ros2
