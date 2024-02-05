// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/logger.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>

#include <string>

namespace spot_ros2 {
/**
 * @brief Implementation of LoggerInterfaceBase that logs messages using rclcpp's logging utilities.
 */
class RclcppLoggerInterface : public LoggerInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppLoggerInterface.
   * @param logger An instance of a logger which will be used to generate ROS 2 logs. This will be copied into the
   * logger_ member.
   */
  explicit RclcppLoggerInterface(const rclcpp::Logger& logger);

  void logDebug(const std::string& message) const override;
  void logInfo(const std::string& message) const override;
  void logWarn(const std::string& message) const override;
  void logError(const std::string& message) const override;
  void logFatal(const std::string& message) const override;

 private:
  /** @brief Logger used to log messages. */
  rclcpp::Logger logger_;
};
}  // namespace spot_ros2
