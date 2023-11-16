// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_robot_api.hpp>
#include <bosdyn/client/gripper_camera_param/gripper_camera_param_client.h>


namespace spot_ros2 {

DefaultSpotApi::DefaultSpotApi(const std::string& sdk_client_name) : client_sdk_{::bosdyn::client::CreateStandardSDK(sdk_client_name)} {}

tl::expected<std::unique_ptr<Robot>, std::string> DefaultSpotApi::createRobot(const std::string& ip_address,
                                                                              const std::string& robot_name) const {
  auto create_robot_result = client_sdk_->CreateRobot(ip_address);
  if (!create_robot_result.status) {
    return tl::make_unexpected("Received error result when creating SDK robot interface: " +
                               create_robot_result.status.DebugString());
  }

  robot_.reset(std::move(create_robot_result.response));

  return {}
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
  time_sync_api_.reset(std::make_shared<DefaultTimeSyncApi>(get_time_sync_thread_response.response));

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

tl::expected<std::unique_ptr<ImageClientApi>, std::string> imageClient() const{
  const auto image_client_result = robot_->EnsureServiceClient<::bosdyn::client::ImageClient>(
      ::bosdyn::client::ImageClient::GetDefaultServiceName());
  if (!image_client_result.status) {
    return tl::make_unexpected("Failed to initialize the Spot SDK image client.");
  }

  return std::make_unique<DefaultImageClientApi>(std::make_unique<::bosdyn::client::ImageClient>(image_client_result.response),
                                                 time_sync_api_,
                                                 hasArm());
}

}  // namespace spot_ros2
