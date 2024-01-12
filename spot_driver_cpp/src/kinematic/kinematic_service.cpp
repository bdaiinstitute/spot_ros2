// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/kinematic_service.hpp>

#include <spot_driver_cpp/conversions/kinematic_conversions.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/kinematic/kinematic_middleware_handle.hpp>

namespace {
constexpr auto kServiceName = "get_inverse_kinematic_solutions";
}

namespace spot_ros2::kinematic {
KinematicService::KinematicService(std::shared_ptr<KinematicApi> kinematicApi,
                                   std::shared_ptr<LoggerInterfaceBase> logger,
                                   std::unique_ptr<MiddlewareHandle> middlewareHandle)
    : kinematicApi_{kinematicApi}, logger_{std::move(logger)}, middlewareHandle_{std::move(middlewareHandle)} {}

void KinematicService::initialize() {
  service_ = middlewareHandle_->createService(
      kServiceName, [this](const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                           std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
        this->getSolutions(request, response);
      });
}

void KinematicService::getSolutions(const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                                    std::shared_ptr<GetInverseKinematicSolutions::Response> response) {
  auto ros_request = request->request;

  bosdyn::api::spot::InverseKinematicsRequest proto_request;
  kinematic_conversions::convertBosdynMsgsInverseKinematicsRequestToProto(ros_request, proto_request);

  auto expected = kinematicApi_->getSolutions(proto_request);
  if (!expected) {
    logger_->logError(std::string{"Error quering the Inverse Kinematics service: "}.append(expected.error()));
    response->response.status.value = bosdyn_msgs::msg::InverseKinematicsResponseStatus::STATUS_UNKNOWN;
  } else {
    kinematic_conversions::convertProtoToBosdynMsgsInverseKinematicsResponse(expected.value().response,
                                                                             response->response);
  }
}
}  // namespace spot_ros2::kinematic
