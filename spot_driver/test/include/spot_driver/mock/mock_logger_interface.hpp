// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/interfaces/logger_interface_base.hpp>

#include <string>

namespace spot_ros2::test {
class MockLoggerInterface : public LoggerInterfaceBase {
 public:
  MOCK_METHOD(void, logDebug, (const std::string& message), (const, override));
  MOCK_METHOD(void, logInfo, (const std::string& message), (const, override));
  MOCK_METHOD(void, logWarn, (const std::string& message), (const, override));
  MOCK_METHOD(void, logError, (const std::string& message), (const, override));
  MOCK_METHOD(void, logFatal, (const std::string& message), (const, override));
};
}  // namespace spot_ros2::test
