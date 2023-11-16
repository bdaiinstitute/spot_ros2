
#include <spot_driver_cpp/api/default_time_sync_api.hpp>

namespace {
builtin_interfaces::msg::Time applyClockSkew(const google::protobuf::Timestamp& timestamp,
                                             const google::protobuf::Duration& clock_skew) {
  int64_t seconds_unskewed = timestamp.seconds() - clock_skew.seconds();
  int32_t nanos_unskewed = timestamp.nanos() - clock_skew.nanos();

  // Carry over a second if needed
  // Note: Since ROS Time messages store the nanoseconds component as an unsigned integer, we need to do this before
  // converting to ROS Time.
  if (nanos_unskewed < 0) {
    nanos_unskewed += 1e9;
    seconds_unskewed -= 1;
  } else if (nanos_unskewed >= 1e9) {
    nanos_unskewed -= 1e9;
    seconds_unskewed += 1;
  }

  // If the timestamp contains a negative time, create an all-zero ROS Time.
  if (seconds_unskewed < 0) {
    return builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
  } else {
    return builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(seconds_unskewed).nanosec(nanos_unskewed);
  }
}
}  // namespace

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
