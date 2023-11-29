// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/image/image_client.h>
#include <bosdyn/client/sdk/client_sdk.h>
#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/api/time_sync_api.hpp>
#include <spot_driver_cpp/images/spot_image_sources.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {
/**
 * @brief Implements ImageClientApi to use the Spot C++ Image Client.
 */
class DefaultImageClientApi : public ImageClientApi {
 public:
  DefaultImageClientApi(::bosdyn::client::ImageClient* image_client, std::shared_ptr<TimeSyncApi> time_sync_api,
                        const std::string& robot_name);
  ~DefaultImageClientApi() = default;

  tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) override;

 private:
  ::bosdyn::client::ImageClient* image_client_;
  std::shared_ptr<TimeSyncApi> time_sync_api_;
  std::string robot_name_;
};
}  // namespace spot_ros2
