// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot_state/robot_state_client.h>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {

class DefaultStateClient : public StateClientInterface {
 public:
  /**
   * @brief constructor for DefaultRobotStateClient.
   *
   * @param client pointer to Spot's RobotStateClient. A DefaultRobotStateClient SHOULD NOT delete this pointer since it
   * does not take ownership.
   * @param time_sync_api A shared_ptr to a time_sync_api. This allows the client to retreive a Spot's current time and
   * clock skew
   * @param robot_name Name of Spot. Used to apply frame_prefix
   */
  explicit DefaultStateClient(::bosdyn::client::RobotStateClient* client);

  tl::expected<bosdyn::api::RobotState, std::string> getRobotState() override;

 private:
  ::bosdyn::client::RobotStateClient* client_;
};

}  // namespace spot_ros2
