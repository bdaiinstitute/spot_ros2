// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>

#include <rclcpp/logging.hpp>

namespace spot_ros2 {
RclcppLoggerInterface::RclcppLoggerInterface(const rclcpp::Logger& logger) : logger_{logger} {}

void RclcppLoggerInterface::logDebug(const std::string& message) const {
  RCLCPP_DEBUG(logger_, "%s", message.c_str());
}

void RclcppLoggerInterface::logInfo(const std::string& message) const {
  RCLCPP_INFO(logger_, "%s", message.c_str());
}

void RclcppLoggerInterface::logWarn(const std::string& message) const {
  RCLCPP_WARN(logger_, "%s", message.c_str());
}

void RclcppLoggerInterface::logError(const std::string& message) const {
  RCLCPP_ERROR(logger_, "%s", message.c_str());
}

void RclcppLoggerInterface::logFatal(const std::string& message) const {
  RCLCPP_FATAL(logger_, "%s", message.c_str());
}
}  // namespace spot_ros2
