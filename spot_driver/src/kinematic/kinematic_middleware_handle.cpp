// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/kinematic/kinematic_middleware_handle.hpp>

namespace spot_ros2::kinematic {

KinematicMiddlewareHandle::KinematicMiddlewareHandle(std::shared_ptr<rclcpp::Node> node) : node_{node} {}

void KinematicMiddlewareHandle::createService(
    std::string service_name, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                 std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                  callback) {
  service_ = node_->create_service<GetInverseKinematicSolutions>(service_name, callback);
}
}  // namespace spot_ros2::kinematic
