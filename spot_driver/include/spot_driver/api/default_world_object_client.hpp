// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/world_object_client_interface.hpp>

#include <bosdyn/client/world_objects/world_object_client.h>
#include <memory>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
/** @brief Implements WorldObjectClientInterface for the Spot API. */
class DefaultWorldObjectClient : public WorldObjectClientInterface {
 public:
  /**
   * @brief Constructor for DefaultWorldObjectClient.
   * @param client Pointer to a WorldObjectClient created by the Spot API.
   */
  explicit DefaultWorldObjectClient(bosdyn::client::WorldObjectClient* client);
  tl::expected<::bosdyn::api::ListWorldObjectResponse, std::string> listWorldObjects(
      ::bosdyn::api::ListWorldObjectRequest& request) const override;
  tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string> mutateWorldObject(
      ::bosdyn::api::MutateWorldObjectRequest& request) const override;

 private:
  /**
   * @brief Pointer to a WorldObjectClient created by the Spot API and passed in during construction.
   * DefaultWorldObjectClient must not delete this pointer, since it does not take ownership.
   */
  bosdyn::client::WorldObjectClient* client_;
};
}  // namespace spot_ros2
