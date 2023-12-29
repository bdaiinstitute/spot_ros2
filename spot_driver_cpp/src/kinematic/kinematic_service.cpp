// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/kinematic_service.hpp>

#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/kinematic/kinematic_middleware_handle.hpp>

namespace {
constexpr auto kServiceName = "get_inverse_kinematic_solutions";
}

namespace spot_ros2::kinematic {
KinematicService::KinematicService(std::shared_ptr<KinematicApi> kinematic_api,
                                   std::shared_ptr<LoggerInterfaceBase> logger,
                                   std::unique_ptr<MiddlewareHandle> middleware_handle)
    : kinematic_api_{kinematic_api}, logger_{std::move(logger)}, middleware_handle_{std::move(middleware_handle)} {}

void KinematicService::initialize() {
  service_ = middleware_handle_->create_service(
      kServiceName, [this](const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                           std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
        this->get_solutions(request, response);
      });
}

void KinematicService::get_solutions(const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                                     std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
  auto ros_request = request->request;

  bosdyn::api::spot::InverseKinematicsRequest proto_request;
  kinematic_conversions::convert_bosdyn_msgs_inverse_kinematics_request_to_proto(ros_request, proto_request);

  auto expected = kinematic_api_->get_solutions(proto_request);
  if (!expected) {
    logger_->logError(std::string{"Error quering the Inverse Kinematics service: "}.append(expected.error()));
    response->response.status.value = bosdyn_msgs::msg::InverseKinematicsResponseStatus::STATUS_UNKNOWN;
  } else {
    kinematic_conversions::convert_proto_to_bosdyn_msgs_inverse_kinematics_response(expected.value().response,
                                                                                    response->response);
  }
}
}  // namespace spot_ros2::kinematic
