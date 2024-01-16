// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <spot_driver_cpp/interfaces/robot_state_client_interface.hpp>

#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>
#include <spot_driver_cpp/interfaces/tf_interface_base.hpp>
#include <spot_driver_cpp/interfaces/timer_interface_base.hpp>

#include <rclcpp/node.hpp>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

/**
 * @brief Retrieves robot state from Spot's robot state service client, converts the messages from Protobuf to ROS
 * messages, and publishes them to the appropriate topics
 */
class SpotRobotStatePublisher {
 public:
  /**
   * @brief A handle that enables dependency injection of ROS and rclcpp::node operations
   */
  class MiddlewareHandle {
   public:
    virtual void createPublishers() = 0;
    virtual void publishRobotState(const RobotState& robot_state) = 0;

    virtual std::shared_ptr<rclcpp::Node> node() = 0;
    virtual ParameterInterfaceBase* parameter_interface() = 0;
    virtual LoggerInterfaceBase* logger_interface() = 0;
    virtual TfInterfaceBase* tf_interface() = 0;
    virtual TimerInterfaceBase* timer_interface() = 0;

    virtual ~MiddlewareHandle() = default;
  };

  /**
   * @brief Constructor for SpotRobotStatePublisher.
   * @details As opposed to other Spot Publishers, initialization takes place inside of the constructor because
   * initialization cannot fail.
   *
   * @param robot_state_client_interface a shared instance of a RobotStateClientInterface.
   * @param middleware_handle A unique instance of a MiddlewareHandle that SpotRobotStatePublisher will take ownership
   * of
   */

  SpotRobotStatePublisher(std::shared_ptr<RobotStateClientInterface> robot_state_client_interface,
                          std::unique_ptr<MiddlewareHandle> middleware_handle);

 private:
  /**
   * @brief Callback function to retrieve and publish Spot's Robot State
   */
  void timerCallback();

  std::string full_odom_frame_id_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<RobotStateClientInterface> client_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;
};

}  // namespace spot_ros2
