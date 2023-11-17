// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_api.hpp>

namespace {
constexpr auto kSDKClientName = "get_image";
}

namespace spot_ros2 {

DefaultRobotApi::DefaultRobotApi() : client_sdk_{::bosdyn::client::CreateStandardSDK(kSDKClientName)} {}

tl::expected<std::unique_ptr<Robot>, std::string> DefaultRobotApi::createRobot(const std::string& ip_address,
                                                                               const std::string& robot_name) const {
  auto create_robot_result = client_sdk_->CreateRobot(ip_address);
  if (!create_robot_result.status) {
    return tl::make_unexpected("Received error result when creating SDK robot interface: " +
                               create_robot_result.status.DebugString());
  }

  return std::make_unique<Robot>(std::move(create_robot_result.response), robot_name);
}
}  // namespace spot_ros2
