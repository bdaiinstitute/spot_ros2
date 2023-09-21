// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/sdk/client_sdk.h>
#include <bosdyn/client/image/image_client.h>
#include <bosdyn/client/time_sync/time_sync_client.h>
#include <builtin_interfaces/msg/time.hpp>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <opencv2/opencv.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/types.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace spot_ros2
{
using GetImagesResult = std::map<ImageSource, ImageWithCameraInfo>;

/**
 * @brief Defines an interface for a class to connect to and interact with Spot.
 */
class SpotInterfaceBase
{
public:
  virtual ~SpotInterfaceBase() {}

  virtual tl::expected<void, std::string> createRobot(const std::string& ip_address, const std::string& robot_name) = 0;
  virtual tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password) = 0;
  virtual tl::expected<bool, std::string> hasArm() const = 0;
  virtual tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) = 0;
  virtual tl::expected<builtin_interfaces::msg::Time, std::string> convertRobotTimeToLocalTime(const google::protobuf::Timestamp& robot_timestamp) = 0;
};

/**
 * @brief Implements SpotInterfaceBase to use the Spot C++ SDK.
 */
class SpotInterface : public SpotInterfaceBase
{
public:
  SpotInterface();

  tl::expected<void, std::string> createRobot(const std::string& ip_address, const std::string& robot_name) override;
  tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password) override;
  tl::expected<bool, std::string> hasArm() const override;
  tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) override;
  tl::expected<builtin_interfaces::msg::Time, std::string> convertRobotTimeToLocalTime(const google::protobuf::Timestamp& robot_timestamp) override;

private:
  tl::expected<google::protobuf::Duration, std::string> getClockSkew();

  std::string robot_name_;
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::shared_ptr<::bosdyn::client::TimeSyncThread> time_sync_thread_;
  std::unique_ptr<::bosdyn::client::ImageClient> image_client_;
};
}
