// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/world_object.pb.h>
#include <bosdyn/client/world_objects/world_object_client.h>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

class WorldObjectClientInterface {
 public:
  WorldObjectClientInterface() = default;
  // Forbid copying or moving classes derived from WorldObjectClientInterface. This is because the implementation of
  // this class for the Spot API contains a pointer to a specific instance of an API client.
  WorldObjectClientInterface(WorldObjectClientInterface&& other) = delete;
  WorldObjectClientInterface(const WorldObjectClientInterface&) = delete;
  WorldObjectClientInterface& operator=(WorldObjectClientInterface&& other) = delete;
  WorldObjectClientInterface& operator=(const WorldObjectClientInterface&) = delete;

  virtual ~WorldObjectClientInterface() = default;

  virtual tl::expected<::bosdyn::api::ListWorldObjectResponse, std::string> listWorldObjects(
      ::bosdyn::api::ListWorldObjectRequest& request) const = 0;

  virtual tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string> mutateWorldObject(
      ::bosdyn::api::MutateWorldObjectRequest& request) const = 0;
};
}  // namespace spot_ros2
