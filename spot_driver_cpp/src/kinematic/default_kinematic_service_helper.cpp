// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/default_kinematic_service_helper.hpp>

namespace spot_ros2 {

DefaultKinematicServiceHelper::DefaultKinematicServiceHelper(std::shared_ptr<rclcpp::Node> node) : node_{node} {}

std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>> DefaultKinematicServiceHelper::create_service(
    std::string service_name, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                 std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                  callback) {
  return node_->create_service<GetInverseKinematicSolutions>(service_name, callback);
}
}  // namespace spot_ros2
