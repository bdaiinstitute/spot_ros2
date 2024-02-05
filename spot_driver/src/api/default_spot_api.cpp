// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/client/gripper_camera_param/gripper_camera_param_client.h>
#include <spot_driver/api/default_image_client.hpp>
#include <spot_driver/api/default_kinematic_api.hpp>
#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>

namespace spot_ros2 {

DefaultSpotApi::DefaultSpotApi(const std::string& sdk_client_name)
    : client_sdk_{::bosdyn::client::CreateStandardSDK(sdk_client_name)} {}

tl::expected<void, std::string> DefaultSpotApi::createRobot(const std::string& ip_address,
                                                            const std::string& robot_name) {
  robot_name_ = robot_name;

  auto create_robot_result = client_sdk_->CreateRobot(ip_address);
  if (!create_robot_result.status) {
    return tl::make_unexpected("Received error result when creating SDK robot interface: " +
                               create_robot_result.status.DebugString());
  }

  robot_ = std::move(create_robot_result.response);

  return {};
}

tl::expected<void, std::string> DefaultSpotApi::authenticate(const std::string& username, const std::string& password) {
  const auto authenticate_result = robot_->Authenticate(username, password);
  if (!authenticate_result) {
    return tl::make_unexpected("Authentication with provided username and password did not succeed.");
  }
  // Start time synchronization between the robot and the client system.
  // This must be done only after a successful authentication.
  const auto start_time_sync_response = robot_->StartTimeSync();
  if (!start_time_sync_response) {
    return tl::make_unexpected("Failed to start time synchronization.");
  }

  const auto get_time_sync_thread_response = robot_->GetTimeSyncThread();
  if (!get_time_sync_thread_response) {
    return tl::make_unexpected("Failed to get the time synchronization thread.");
  }
  time_sync_api_ = std::make_shared<DefaultTimeSyncApi>(get_time_sync_thread_response.response);

  // Image API.

  const auto image_client_result = robot_->EnsureServiceClient<::bosdyn::client::ImageClient>(
      ::bosdyn::client::ImageClient::GetDefaultServiceName());
  if (!image_client_result.status) {
    return tl::make_unexpected("Failed to create Image client.");
  }
  image_client_interface_ =
      std::make_shared<DefaultImageClient>(image_client_result.response, time_sync_api_, robot_name_);

  // Kinematic API.
  const auto kinematic_api_result = robot_->EnsureServiceClient<::bosdyn::client::InverseKinematicsClient>(
      ::bosdyn::client::InverseKinematicsClient::GetDefaultServiceName());
  if (!kinematic_api_result.status) {
    return tl::make_unexpected("Failed to create Inverse Kinematic client.");
  }
  kinematicApi_ = std::make_shared<DefaultKinematicApi>(kinematic_api_result.response);

  return {};
}

tl::expected<bool, std::string> DefaultSpotApi::hasArm() const {
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

std::shared_ptr<ImageClientInterface> DefaultSpotApi::image_client_interface() const {
  return image_client_interface_;
}
std::shared_ptr<KinematicApi> DefaultSpotApi::kinematicApi() const {
  return kinematicApi_;
}
}  // namespace spot_ros2
