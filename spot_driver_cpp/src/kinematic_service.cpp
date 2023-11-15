// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic_service.hpp>

namespace spot_ros2 {
KinematicService::KinematicService(std::unique_ptr<KinematicApi> kinematic_api, std::shared_ptr<rclcpp::Node> node)
    : kinematic_api_{std::move(kinematic_api)} {}
}  // namespace spot_ros2