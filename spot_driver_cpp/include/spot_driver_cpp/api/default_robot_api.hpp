
// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/robot_api.hpp>

#include <bosdyn/client/sdk/client_sdk.h>

namespace spot_ros2 {

class DefaultRobotApi : public RobotApi {
 public:
  DefaultRobotApi();
  tl::expected<std::unique_ptr<Robot>, std::string> createRobot(const std::string& ip_address,
                                                                const std::string& robot_name) const override;

 private:
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
};
}  // namespace spot_ros2
