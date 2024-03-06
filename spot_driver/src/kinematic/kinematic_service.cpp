// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/kinematic/kinematic_service.hpp>

#include <spot_driver/conversions/kinematic_conversions.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/kinematic/kinematic_middleware_handle.hpp>

namespace {
constexpr auto kServiceName = "get_inverse_kinematic_solutions";
}

namespace spot_ros2::kinematic {
KinematicService::KinematicService(std::shared_ptr<KinematicApi> kinematic_api,
                                   std::shared_ptr<LoggerInterfaceBase> logger,
                                   std::unique_ptr<MiddlewareHandle> middleware_handle)
    : kinematic_api_{kinematic_api}, logger_{std::move(logger)}, middleware_handle_{std::move(middleware_handle)} {}

void KinematicService::initialize() {
  middleware_handle_->createService(kServiceName,
                                    [this](const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                                           std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
                                      this->getSolutions(request, response);
                                    });
}

void KinematicService::getSolutions(const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                                    std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
  auto ros_request = request->request;

  bosdyn::api::spot::InverseKinematicsRequest proto_request;
  convertToProto(ros_request, proto_request);

  auto expected = kinematic_api_->getSolutions(proto_request);
  if (!expected) {
    logger_->logError(std::string{"Error querying the Inverse Kinematics service: "}.append(expected.error()));
    response->response.status.value = bosdyn_spot_api_msgs::msg::InverseKinematicsResponseStatus::STATUS_UNKNOWN;
  } else {
    convertToRos(expected.value(), response->response);
  }
}
}  // namespace spot_ros2::kinematic
