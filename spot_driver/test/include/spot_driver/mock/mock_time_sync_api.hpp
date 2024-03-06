// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/time.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2::test {
class MockTimeSyncApi : public TimeSyncApi {
 public:
  MOCK_METHOD((tl::expected<google::protobuf::Duration, std::string>), getClockSkew, (), (override));
};
}  // namespace spot_ros2::test
