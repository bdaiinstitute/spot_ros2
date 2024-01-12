// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/kinematic_api.hpp>

#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>

#include <spot_msgs/srv/get_inverse_kinematic_solutions.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/service.hpp>

#include <memory>
#include <string>

namespace spot_ros2::kinematic {

using spot_msgs::srv::GetInverseKinematicSolutions;

class KinematicService {
 public:
  /**
   * @brief A handle class around rclcpp::Node operations for KinematicService
   */
  class MiddlewareHandle {
   public:
    virtual std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>> createService(
        std::string serviceName, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                    std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                     callback) = 0;
    virtual ~MiddlewareHandle() = default;
  };

  /**
   * Create the logic for the GetInverseKinematicSolutions service.
   * @param node The ROS node.
   * @param kinematicApi The Api to interact with the Spot SDK.
   * @param logger Logging interface.
   */
  explicit KinematicService(std::shared_ptr<KinematicApi> kinematicApi, std::shared_ptr<LoggerInterfaceBase> logger,
                            std::unique_ptr<MiddlewareHandle> middlewareHandle);

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
  std::shared_ptr<KinematicApi> kinematicApi_;

  // Logger.
  std::shared_ptr<LoggerInterfaceBase> logger_;

  // The service provider.
  std::unique_ptr<MiddlewareHandle> middlewareHandle_;

  // The service processing the Inverse Kinematic request for solutions.
  rclcpp::Service<GetInverseKinematicSolutions>::SharedPtr service_;
};
}  // namespace spot_ros2::kinematic
