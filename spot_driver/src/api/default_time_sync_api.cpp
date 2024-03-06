// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/api/default_time_sync_api.hpp>

namespace spot_ros2 {

DefaultTimeSyncApi::DefaultTimeSyncApi(std::shared_ptr<::bosdyn::client::TimeSyncThread> time_sync_thread)
    : time_sync_thread_{time_sync_thread} {}

tl::expected<google::protobuf::Duration, std::string> DefaultTimeSyncApi::getClockSkew() {
  if (!time_sync_thread_) {
    return tl::make_unexpected("Time sync thread was not initialized.");
  }
  const auto get_skew_response = time_sync_thread_->GetEndpoint()->GetClockSkew();
  if (!get_skew_response) {
    return tl::make_unexpected("Received a failure result from the TimeSyncEndpoint: " +
                               get_skew_response.status.DebugString());
  }
  return *get_skew_response.response;
}

}  // namespace spot_ros2
