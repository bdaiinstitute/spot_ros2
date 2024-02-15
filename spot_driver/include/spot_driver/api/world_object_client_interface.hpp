// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/world_object.pb.h>
#include <bosdyn/client/world_objects/world_object_client.h>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

class WorldObjectClientInterface {
 public:
  virtual ~WorldObjectClientInterface() = default;

  virtual tl::expected<::bosdyn::api::ListWorldObjectResponse, std::string> listWorldObjects(
      ::bosdyn::api::ListWorldObjectRequest& request) = 0;

  virtual tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string> mutateWorldObject(
      ::bosdyn::api::MutateWorldObjectRequest& request) = 0;
};
}  // namespace spot_ros2
