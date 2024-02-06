// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/kinematic_api.hpp>

#include <spot_driver/interfaces/logger_interface_base.hpp>

#include <spot_msgs/srv/get_inverse_kinematic_solutions.hpp>

#include <memory>
#include <string>

namespace spot_ros2::kinematic {

using spot_msgs::srv::GetInverseKinematicSolutions;

class KinematicService {
 public:
  /**
   * This middleware handle is used to register a service and assign to it a
   * callback. In testing, it can be mocked to avoid using the ROS
   * infrastructure.
   */
  class MiddlewareHandle {
   public:
    virtual void createService(std::string service_name,
                               std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                  std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                   callback) = 0;
    virtual ~MiddlewareHandle() = default;
  };

  /**
   * Create the logic for the GetInverseKinematicSolutions service.
   * @param node The ROS node.
   * @param kinematic_api The Api to interact with the Spot SDK.
   * @param logger Logging interface.
   */
  explicit KinematicService(std::shared_ptr<KinematicApi> kinematic_api, std::shared_ptr<LoggerInterfaceBase> logger,
                            std::unique_ptr<MiddlewareHandle> middleware_Handle);

  /** Initialize the service. */
  void initialize();

  /**
   * Invoke the Spot SDK to get IK solutions.
   * @param request The ROS request.
   * @param response A ROS response to be filled.
   */
  void getSolutions(const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                    std::shared_ptr<GetInverseKinematicSolutions::Response> response);

 private:
  // The API to interact with Spot SDK.
  std::shared_ptr<KinematicApi> kinematic_api_;

  // Logger.
  std::shared_ptr<LoggerInterfaceBase> logger_;

  // The service provider.
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
};
}  // namespace spot_ros2::kinematic
