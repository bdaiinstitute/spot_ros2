// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/robot.hpp>

#include <bosdyn/client/gripper_camera_param/gripper_camera_param_client.h>

namespace spot_ros2 {
Robot::Robot(std::unique_ptr<::bosdyn::client::Robot> robot, const std::string& robot_name)
    : robot_{std::move(robot)}, robot_name_{robot_name} {}

std::string Robot::name() const {
  return robot_name_;
}

tl::expected<bool, std::string> Robot::hasArm() const {
  // Determine if Spot has an arm by checking if the client for gripper camera parameters exists, since Spots without
  // arms do not have this client.
  const auto list_result = robot_->ListServices();
  if (!list_result.status) {
    return tl::make_unexpected("Failed to retrieve list of Spot services.");
  }

  const auto& services = list_result.response;

  return std::find_if(services.cbegin(), services.cend(), [](const ::bosdyn::api::ServiceEntry& entry) {
           return entry.name() == ::bosdyn::client::GripperCameraParamClient::GetDefaultServiceName();
         }) != services.cend();
}

tl::expected<void, std::string> Robot::authenticate(const std::string& username, const std::string& password) {
  const auto authenticate_result = robot_->Authenticate(username, password);
  if (!authenticate_result) {
    return tl::make_unexpected("Authentication with provided username and password did not succeed.");
  }
  return {};
}

::bosdyn::client::Robot& Robot::robot() const {
  return *robot_;
}

}  // namespace spot_ros2
