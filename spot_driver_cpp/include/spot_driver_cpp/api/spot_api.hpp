// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/api/kinematic_api.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {

class SpotApi {
 public:
  virtual ~SpotApi() = default;

  virtual tl::expected<void, std::string> createRobot(const std::string& ip_address, const std::string& robot_name) = 0;
  virtual tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password) = 0;
  virtual tl::expected<bool, std::string> hasArm() const = 0;
  virtual std::shared_ptr<ImageClientApi> image_client_api() const = 0;
  virtual std::shared_ptr<KinematicApi> kinematic_api() const = 0;
};
}  // namespace spot_ros2
