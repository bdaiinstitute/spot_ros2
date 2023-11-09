// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_api.hpp>

namespace {
constexpr auto kSDKClientName = "get_image";
}

namespace spot_ros2 {

DefaultRobotApi::DefaultRobotApi() : client_sdk_{::bosdyn::client::CreateStandardSDK(kSDKClientName)} {}

tl::expected<std::unique_ptr<::bosdyn::client::Robot>, std::string> DefaultRobotApi::createRobot(
    const std::string& ip_address, const std::string& robot_name) {
  auto create_robot_result = client_sdk_->CreateRobot(ip_address);
  if (!create_robot_result.status) {
    return tl::make_unexpected("Received error result when creating SDK robot interface: " +
                               create_robot_result.status.DebugString());
  }

  return std::make_unique<::bosdyn::client::Robot>(create_robot_result.response);
}
}  // namespace spot_ros2
