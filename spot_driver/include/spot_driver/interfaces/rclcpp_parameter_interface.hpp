// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>

#include <memory>
#include <string>

namespace spot_ros2 {
/**
 * @brief Implements ParameterInterfaceBase to declare and retrieve ROS 2 parameters.
 */
class RclcppParameterInterface : public ParameterInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppParameterInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppParameterInterface shares ownership of the shared_ptr.
   */
  explicit RclcppParameterInterface(const std::shared_ptr<rclcpp::Node>& node);
  std::string getHostname() const override;
  std::string getUsername() const override;
  std::string getPassword() const override;
  double getRGBImageQuality() const override;
  bool getHasRGBCameras() const override;
  bool getPublishRGBImages() const override;
  bool getPublishDepthImages() const override;
  bool getPublishDepthRegisteredImages() const override;
  std::string getPreferredOdomFrame() const override;
  std::string getSpotName() const override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace spot_ros2
