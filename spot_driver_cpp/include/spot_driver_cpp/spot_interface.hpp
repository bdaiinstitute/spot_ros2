// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/sdk/client_sdk.h>
#include <bosdyn/client/image/image_client.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tl_expected/expected.hpp>

#include <unordered_map>

namespace spot_ros2
{
using GetImagesResult = std::unordered_map<std::string, sensor_msgs::msg::Image>;

/**
 * @brief Defines an interface for a class to connect to and interact with Spot.
 */
class SpotInterfaceBase
{
public:
  virtual ~SpotInterfaceBase() {}

  virtual bool createRobot(const std::string& ip_address) = 0;
  virtual bool authenticate(const std::string& username, const std::string& password) = 0;
  virtual bool hasArm() const = 0;
  virtual tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) = 0;
};

/**
 * @brief Implements SpotInterfaceBase to use the Spot C++ SDK.
 */
class SpotInterface : public SpotInterfaceBase
{
public:
  SpotInterface();

  bool createRobot(const std::string& ip_address) override;
  bool authenticate(const std::string& username, const std::string& password) override;
  bool hasArm() const override;
  tl::expected<GetImagesResult, std::string> getImages(::bosdyn::api::GetImageRequest request) override;

private:
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::unique_ptr<::bosdyn::client::ImageClient> image_client_;
};
}
