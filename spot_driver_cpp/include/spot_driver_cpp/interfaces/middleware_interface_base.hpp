// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>
#include <spot_driver_cpp/interfaces/publisher_interface_base.hpp>
#include <spot_driver_cpp/interfaces/tf_interface_base.hpp>
#include <spot_driver_cpp/interfaces/timer_interface_base.hpp>

#include <optional>
#include <string>

namespace spot_ros2 {

class MiddlewareInterface {
 public:
  virtual ParameterInterfaceBase* parameter_interface() = 0;
  virtual LoggerInterfaceBase* logger_interface() = 0;
  virtual PublisherInterfaceBase* publisher_interface() = 0;
  virtual TfInterfaceBase* tf_interface() = 0;
  virtual TimerInterfaceBase* timer_interface() = 0;
};

}  // namespace spot_ros2
