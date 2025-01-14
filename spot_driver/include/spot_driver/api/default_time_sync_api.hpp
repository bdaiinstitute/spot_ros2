// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/time_sync/time_sync_helpers.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/time.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <tl_expected/expected.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace spot_ros2 {

class DefaultTimeSyncApi : public TimeSyncApi {
 public:
  explicit DefaultTimeSyncApi(std::shared_ptr<::bosdyn::client::TimeSyncThread> time_sync_thread,
                              const std::chrono::seconds timesync_timeout);

  /**
  * @brief Get the current clock skew from the Spot SDK's time sync endpoint.
  * @details The clock skew is the difference between Spot's internal system clock and the host PC's system clock.
  * For example, a clock skew of 1.0 seconds means that a timestamp from Spot's clock that is 10.0 seconds after epoch
  * would correspond to a timestmap from the system clock that is 9.0 seconds after epoch.

  * The Spot SDK documentation provides a more detailed explanation of how Spot's time sync works here:
  * https://dev.bostondynamics.com/docs/concepts/base_services#time-sync
  *
  * @return If the clock skew was successfully calculated, return a Duration containing the difference between Spot's
  * internal clock and the host's system clock.
  * @return If the Spot SDK's time sync thread was not initialized, return an error message.
  * @return If the Spot SDK's time sync endpoint fails to handle the clock skew request, return an error message.
  */
  [[nodiscard]] tl::expected<google::protobuf::Duration, std::string> getClockSkew() override;

 private:
  std::shared_ptr<::bosdyn::client::TimeSyncThread> time_sync_thread_;
  const std::chrono::seconds timesync_timeout_;
};
}  // namespace spot_ros2
