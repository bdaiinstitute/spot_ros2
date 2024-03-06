// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <optional>
#include <string>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that retrieves the user-configured parameters needed to connect to Spot.
 */
class ParameterInterfaceBase {
 public:
  virtual ~ParameterInterfaceBase() {}

  // These functions retrieve optional parameters, where a default value can be used if the user does not provide a
  // specific value. If the parameter was set, return the value provided by the user. If the parameter was not set,
  // return the default value.
  virtual std::string getHostname() const = 0;
  virtual std::string getUsername() const = 0;
  virtual std::string getPassword() const = 0;
  virtual double getRGBImageQuality() const = 0;
  virtual bool getHasRGBCameras() const = 0;
  virtual bool getPublishRGBImages() const = 0;
  virtual bool getPublishDepthImages() const = 0;
  virtual bool getPublishDepthRegisteredImages() const = 0;
  virtual std::string getPreferredOdomFrame() const = 0;
  virtual std::string getSpotName() const = 0;

 protected:
  // These are the definitions of the default values for optional parameters.
  static constexpr auto kDefaultHostname = "10.0.0.3";
  static constexpr auto kDefaultUsername = "user";
  static constexpr auto kDefaultPassword = "password";
  static constexpr double kDefaultRGBImageQuality{70.0};
  static constexpr bool kDefaultHasRGBCameras{true};
  static constexpr bool kDefaultPublishRGBImages{true};
  static constexpr bool kDefaultPublishDepthImages{true};
  static constexpr bool kDefaultPublishDepthRegisteredImages{true};
  static constexpr auto kDefaultPreferredOdomFrame = "odom";
};
}  // namespace spot_ros2
