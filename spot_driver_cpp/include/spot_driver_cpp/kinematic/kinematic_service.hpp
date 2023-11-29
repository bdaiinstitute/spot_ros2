// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/kinematic_api.hpp>

#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/kinematic/kinematic_service_helper.hpp>

#include <spot_msgs/srv/get_inverse_kinematic_solutions.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/service.hpp>

#include <memory>

namespace spot_ros2 {

using spot_msgs::srv::GetInverseKinematicSolutions;

class KinematicService {
 public:
  /**
   * Create the logic for the GetInverseKinematicSolutions service.
   * @param node The ROS node.
   * @param kinematic_api The Api to interact with the Spot SDK.
   * @param logger Logging interface.
   */
  explicit KinematicService(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<KinematicApi> kinematic_api,
                            std::unique_ptr<LoggerInterfaceBase> logger,
                            std::unique_ptr<KinematicServiceHelper> service_helper);

  /**
   * Create the logic for the GetInverseKinematicSolutions service.
   * @param node The ROS node.
   * @param kinematic_api The Api to interact with the Spot SDK.
   */
  explicit KinematicService(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<KinematicApi> kinematic_api);

  /** Initialize the service. */
  void init();

 private:
  // The API to interact with Spot SDK.
  std::shared_ptr<KinematicApi> kinematic_api_;

  // The service provider.
  std::unique_ptr<KinematicServiceHelper> service_helper_;

  // The service processing the Inverse Kinematic request for solutions.
  rclcpp::Service<GetInverseKinematicSolutions>::SharedPtr service_;

  // Logger.
  std::unique_ptr<LoggerInterfaceBase> logger_;

  // The service callback.
  void service_callback_(const std::shared_ptr<GetInverseKinematicSolutions::Request> request,
                         std::shared_ptr<GetInverseKinematicSolutions::Response> response);
};
}  // namespace spot_ros2