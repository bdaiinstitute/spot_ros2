// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/robot_state/robot_state_client.h>
#include <spot_driver/api/state_client_interface.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

class DefaultStateClient final : public StateClientInterface {
 public:
  /**
   * @brief constructor for DefaultStateClient.
   *
   * @param client A pointer to Spot's RobotStateClient. A DefaultStateClient SHOULD NOT delete this pointer since it
   * does not take ownership.
   */
  explicit DefaultStateClient(::bosdyn::client::RobotStateClient* client);

  /**
   * @brief Retrieve Spot's most recent robot state data.
   * @return Returns an expected which contains a RobotState message if the request was completed successfully or an
   * error message if the request could not be completed.
   */
  [[nodiscard]] tl::expected<bosdyn::api::RobotState, std::string> getRobotState() override;

 private:
  /** @brief A pointer to a RobotStateClient provided to this class during construction. */
  ::bosdyn::client::RobotStateClient* client_;
};

}  // namespace spot_ros2
