// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/spot_api.hpp>

#include <bosdyn/client/robot/Robot.h>
#include <bosdyn/client/sdk/client_sdk.h>

#include <memory>
#include <string>

namespace spot_ros2 {

class DefaultSpotApi : public SpotApi {
 public:
  explicit DefaultSpotApi(const std::string& sdk_client_name);
  tl::expected<void, std::string> createRobot(const std::string& ip_address,
                                              const std::string& robot_name) const override;
  tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password) override;

  tl::expected<bool, std::string> hasArm() const override;

  tl::expected<std::unique_ptr<ImageClientApi>, std::string> imageClient() const override;
  std::shared_ptr<TimeSyncApi> time_sync_api() override;

 private:
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::shared_ptr<TimeSyncApi> time_sync_api_;
};
}  // namespace spot_ros2
