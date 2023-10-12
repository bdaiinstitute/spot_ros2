// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>

namespace spot_ros2
{
/**
 * @brief Implements ParameterInterfaceBase to declare and retrieve ROS 2 parameters.
 */
class RclcppParameterInterface : public ParameterInterfaceBase
{
public:
  explicit RclcppParameterInterface(const std::shared_ptr<rclcpp::Node>& node);
  std::optional<std::string> getAddress() const override;
  std::optional<std::string> getUsername() const override;
  std::optional<std::string> getPassword() const override;
  double getRGBImageQuality() const override;
  bool getHasRGBCameras() const override;
  bool getPublishRGBImages() const override;
  bool getPublishDepthImages() const override;
  bool getPublishDepthRegisteredImages() const override;
  std::string getSpotName() const override;

private:
  std::shared_ptr<rclcpp::Node> node_;
};
}
