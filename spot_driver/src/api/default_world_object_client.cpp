// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/api/default_world_object_client.hpp>

#include <bosdyn/api/world_object.pb.h>
#include <bosdyn/client/world_objects/world_object_client.h>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

DefaultWorldObjectClient::DefaultWorldObjectClient(bosdyn::client::WorldObjectClient* client) : client_{client} {}

tl::expected<::bosdyn::api::ListWorldObjectResponse, std::string> DefaultWorldObjectClient::listWorldObjects(
    ::bosdyn::api::ListWorldObjectRequest& request) const {
  try {
    auto result = client_->ListWorldObjects(request);
    if (result) {
      return result.response;
    }
    return tl::make_unexpected("The ListWorldObjects service returned with error code " +
                               std::to_string(result.status.code().value()) + ": " + result.status.message());
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to query the ListWorldObjects service: " + std::string{ex.what()});
  }
}

tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string> DefaultWorldObjectClient::mutateWorldObject(
    ::bosdyn::api::MutateWorldObjectRequest& request) const {
  try {
    auto result = client_->MutateWorldObjects(request);
    if (result) {
      return result.response;
    }
    return tl::make_unexpected("The MutateWorldObjects service returned with error code " +
                               std::to_string(result.status.code().value()) + ": " + result.status.message());
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to query the MutateWorldObjects service: " + std::string{ex.what()});
  }
}
}  // namespace spot_ros2
