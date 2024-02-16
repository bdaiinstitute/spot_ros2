// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/world_object_client_interface.hpp>

#include <bosdyn/client/world_objects/world_object_client.h>
#include <memory>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
class DefaultWorldObjectClient : public WorldObjectClientInterface {
 public:
  explicit DefaultWorldObjectClient(std::unique_ptr<bosdyn::client::WorldObjectClient> client);
  tl::expected<::bosdyn::api::ListWorldObjectResponse, std::string> listWorldObjects(
      ::bosdyn::api::ListWorldObjectRequest& request) override;
  tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string> mutateWorldObject(
      ::bosdyn::api::MutateWorldObjectRequest& request) override;

 private:
  std::unique_ptr<bosdyn::client::WorldObjectClient> client_;
};
}  // namespace spot_ros2
