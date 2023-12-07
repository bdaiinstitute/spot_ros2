// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot_state/robot_state_client.h>
#include <spot_driver_cpp/api/time_sync_api.hpp>
#include <spot_driver_cpp/interfaces/robot_state_client_interface.hpp>

#include <string>
#include <memory>

namespace spot_ros2 {

class DefaultRobotStateClient : public RobotStateClientInterface {
 public:
  explicit DefaultRobotStateClient(::bosdyn::client::RobotStateClient* client,
                                   std::shared_ptr<TimeSyncApi> time_sync_api, const std::string& robot_name);

  tl::expected<RobotState, std::string> getRobotState() override;

 private:
  ::bosdyn::client::RobotStateClient* client_;
  std::shared_ptr<TimeSyncApi> time_sync_api_;
  std::string frame_prefix_;
  std::string preferred_odom_frame_;
};

}  // namespace spot_ros2
