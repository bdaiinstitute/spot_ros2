// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/kinematic_middleware_handle.hpp>

namespace spot_ros2::kinematic {

KinematicMiddlewareHandle::KinematicMiddlewareHandle(std::shared_ptr<rclcpp::Node> node) : node_{node} {}

std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>> KinematicMiddlewareHandle::create_service(
    std::string service_name, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                 std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                  callback) {
  return node_->create_service<GetInverseKinematicSolutions>(service_name, callback);
}
}  // namespace spot_ros2::kinematic