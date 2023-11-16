// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/image/image_client.h>
#include <bosdyn/client/sdk/client_sdk.h>
#include <bosdyn/client/time_sync/time_sync_client.h>

#include <spot_driver_cpp/api/image_api.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {
/**
 * @brief Implements ImageClientApi to use the Spot C++ Image Client.
 */
class DefaultImageClientApi : public ImageClientApi {
 public:
  DefaultImageClientApi(std::unique_ptr<::bosdyn::client::ImageClient> image_client,
                        std::shared_ptr<TimeSyncApi> time_sync_api, const std::string& robot_name);
  ~DefaultImageClientApi() = default;

  tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) override;

 private:
  std::shared_ptr<TimeSyncApi> time_sync_api_;
  std::unique_ptr<::bosdyn::client::ImageClient> image_client_;
  std::string robot_name_;
};
}  // namespace spot_ros2
