// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <string>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that logs messages at different severity levels.
 */
class LoggerInterfaceBase {
 public:
  virtual ~LoggerInterfaceBase() {}

  virtual void logDebug(const std::string& message) const = 0;
  virtual void logInfo(const std::string& message) const = 0;
  virtual void logWarn(const std::string& message) const = 0;
  virtual void logError(const std::string& message) const = 0;
  virtual void logFatal(const std::string& message) const = 0;
};
}  // namespace spot_ros2
