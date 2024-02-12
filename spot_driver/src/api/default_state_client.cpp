// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <spot_driver/api/default_state_client.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>

namespace spot_ros2 {

DefaultStateClient::DefaultStateClient(::bosdyn::client::RobotStateClient* client,
                                       const std::shared_ptr<TimeSyncApi>& time_sync_api, const std::string& robot_name)
    : client_{client}, time_sync_api_{time_sync_api}, frame_prefix_{robot_name.empty() ? "" : robot_name + "/"} {}

tl::expected<RobotState, std::string> DefaultStateClient::getRobotState(const std::string& preferred_odom_frame) {
  std::shared_future<::bosdyn::client::RobotStateResultType> get_robot_state_result_future =
      client_->GetRobotStateAsync();

  const ::bosdyn::client::RobotStateResultType& get_robot_state_result = get_robot_state_result_future.get();
  if (!get_robot_state_result.status || !get_robot_state_result.response.has_robot_state()) {
    return tl::make_unexpected("Failed to get robot state: " + get_robot_state_result.status.DebugString());
  }

  const auto clock_skew_result = time_sync_api_->getClockSkew();
  if (!clock_skew_result) {
    return tl::make_unexpected("Failed to get latest clock skew: " + clock_skew_result.error());
  }

  const auto robot_state = get_robot_state_result.response.robot_state();

  return RobotState{
      getBatteryStates(robot_state, clock_skew_result.value()),
      getWifiState(robot_state),
      getFootState(robot_state),
      getEstopStates(robot_state, clock_skew_result.value()),
      getJointStates(robot_state, clock_skew_result.value(), frame_prefix_),
      getTf(robot_state, clock_skew_result.value(), frame_prefix_, preferred_odom_frame),
      getOdomTwist(robot_state, clock_skew_result.value()),
      getOdom(robot_state, clock_skew_result.value(), frame_prefix_, preferred_odom_frame == frame_prefix_ + "vision"),
      getPowerState(robot_state, clock_skew_result.value()),
      getSystemFaultState(robot_state, clock_skew_result.value()),
      getManipulatorState(robot_state),
      getEndEffectorForce(robot_state, clock_skew_result.value(), frame_prefix_),
      getBehaviorFaultState(robot_state, clock_skew_result.value())};
}

}  // namespace spot_ros2
