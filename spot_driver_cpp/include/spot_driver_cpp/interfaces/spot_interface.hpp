// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/image/image_client.h>
#include <bosdyn/client/sdk/client_sdk.h>
#include <bosdyn/client/time_sync/time_sync_client.h>
#include <spot_driver_cpp/interfaces/spot_interface_base.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {
/**
 * @brief Implements SpotInterfaceBase to use the Spot C++ SDK.
 */
class SpotInterface : public SpotInterfaceBase {
 public:
  SpotInterface();

  tl::expected<void, std::string> createRobot(const std::string& ip_address, const std::string& robot_name) override;
  tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password) override;
  tl::expected<bool, std::string> hasArm() const override;
  tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) override;
  tl::expected<builtin_interfaces::msg::Time, std::string> convertRobotTimeToLocalTime(
      const google::protobuf::Timestamp& robot_timestamp) override;

 private:
  /**
  * @brief Get the current clock skew from the Spot SDK's time sync endpoint.
  * @details The clock skew is the difference between Spot's internal system clock and the host PC's system clock.
  * For example, a clock skew of 1.0 seconds means that a timestamp from Spot's clock that is 10.0 seconds after epoch
  * would correspond to a timestmap from the system clock that is 9.0 seconds after epoch.

  * The Spot SDK documentation provides a more detailed explanation of how Spot's time sync works here:
  * https://dev.bostondynamics.com/docs/concepts/base_services#time-sync
  *
  * @return If the clock skew was successfully calculated, return a Duration containing the difference between Spot's
  * internal clock and the host's system clodk.
  * @return If the Spot SDK's time sync thread was not initialized, return an error message.
  * @return If the Spot SDK's time sync endpoint fails to handle the clock skew request, return an error message.
  */
  tl::expected<google::protobuf::Duration, std::string> getClockSkew();

  std::string robot_name_;
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::shared_ptr<::bosdyn::client::TimeSyncThread> time_sync_thread_;
  std::unique_ptr<::bosdyn::client::ImageClient> image_client_;
};
}  // namespace spot_ros2
