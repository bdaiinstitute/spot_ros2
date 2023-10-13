// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/logger.hpp>
#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>

namespace spot_ros2
{
/**
 * @brief Implementation of LoggerInterfaceBase that logs messages using rclcpp's logging utilities.
*/
class RclcppLoggerInterface : public LoggerInterfaceBase
{
public:
  explicit RclcppLoggerInterface(const rclcpp::Logger& logger);

  void logDebug(const std::string& message) const override;
  void logInfo(const std::string& message) const override;
  void logWarn(const std::string& message) const override;
  void logError(const std::string& message) const override;
  void logFatal(const std::string& message) const override;

private:
  rclcpp::Logger logger_;
};
}
