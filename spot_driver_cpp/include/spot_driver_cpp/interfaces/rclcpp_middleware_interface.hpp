// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/interfaces/middleware_interface_base.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_publisher_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_tf_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_wall_timer_interface.hpp>

#include <optional>
#include <string>

namespace spot_ros2 {

class RclcppMiddlewareInterface : public MiddlewareInterface{
 public:
  RclcppMiddlewareInterface(std::shared_ptr<rclcpp::Node> node)
    : parameter_interface_{std::make_unique<RclcppParameterInterface>(node)},
      logger_interface_{std::make_unique<RclcppLoggerInterface>(node->get_logger())},
      publisher_interface_{std::make_unique<RclcppPublisherInterface>(node)},
      tf_interface_{std::make_unique<RclcppTfInterface>(node)},
      timer_interface_{std::make_unique<RclcppWallTimerInterface>(node)} {}

  ParameterInterfaceBase* parameter_interface(){
    return parameter_interface_.get();
  }
  LoggerInterfaceBase* logger_interface(){
    return logger_interface_.get();
  }
  PublisherInterfaceBase* publisher_interface(){
    return publisher_interface_.get();
  }
  TfInterfaceBase* tf_interface(){
    return tf_interface_.get();
  }
  TimerInterfaceBase* timer_interface(){
    return timer_interface_.get();
  }
 
 private:
  std::unique_ptr<RclcppParameterInterface> parameter_interface_;
  std::unique_ptr<RclcppLoggerInterface> logger_interface_;
  std::unique_ptr<RclcppPublisherInterface> publisher_interface_;
  std::unique_ptr<RclcppTfInterface> tf_interface_;
  std::unique_ptr<RclcppWallTimerInterface> timer_interface_;
};

} // namespace spot_ros2
