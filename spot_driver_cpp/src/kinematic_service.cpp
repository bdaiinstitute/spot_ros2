// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic_service.hpp>

#include <spot_driver_cpp/kinematic_utils.hpp>

namespace spot_ros2 {

auto kServiceName = "get_inverse_kinematic_solutions";

KinematicService::KinematicService(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<KinematicApi> kinematic_api)
    : node_{node}, kinematic_api_{kinematic_api} {}

void KinematicService::init() {
  service_ = node_->create_service<GetInverseKinematicSolutions>(
      kServiceName, [this](const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                           std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
        this->service_callback_(request, response);
      });
}

void KinematicService::service_callback_(const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                                         std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
  auto proto_request = kinematic_utils::to_proto(request);

  // const ::bosdyn::api::spot::InverseKinematicsRequest proto_request;
  // auto result = kinematic_api_->get_solutions(proto_request);
}
}  // namespace spot_ros2
