// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <bosdyn/api/robot_state.pb.h>
#include <spot_driver/api/world_object_client_interface.hpp>
#include <string>

namespace spot_ros2::test {
class MockWorldObjectClient : public WorldObjectClientInterface {
 public:
  MOCK_METHOD((tl::expected<::bosdyn::api::ListWorldObjectResponse, std::string>), listWorldObjects,
              (::bosdyn::api::ListWorldObjectRequest & request), (const, override));

  MOCK_METHOD((tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string>), mutateWorldObject,
              (::bosdyn::api::MutateWorldObjectRequest & request), (const, override));
};
}  // namespace spot_ros2::test
