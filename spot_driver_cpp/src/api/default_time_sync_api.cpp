
#include <spot_driver_cpp/api/default_time_sync_api.hpp>

namespace spot_ros2 {

DefaultTimeSyncApi::DefaultTimeSyncApi(std::shared_ptr<::bosdyn::client::TimeSyncThread> time_sync_thread)
    : time_sync_thread_{time_sync_thread} {}

tl::expected<builtin_interfaces::msg::Time, std::string> DefaultTimeSyncApi::convertRobotTimeToLocalTime(
    const google::protobuf::Timestamp& robot_timestamp) {
  const auto get_clock_skew_result = getClockSkew();
  if (!get_clock_skew_result) {
    return tl::make_unexpected("Failed to get clock skew: " + get_clock_skew_result.error());
  }

  return applyClockSkew(robot_timestamp, get_clock_skew_result.value());
}

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
