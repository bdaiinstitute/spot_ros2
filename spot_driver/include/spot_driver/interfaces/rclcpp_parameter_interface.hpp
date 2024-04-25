// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>

#include <memory>
#include <optional>
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
  [[nodiscard]] std::string getHostname() const override;
  [[nodiscard]] std::optional<int> getPort() const override;
  [[nodiscard]] std::optional<std::string> getCertificate() const override;
  [[nodiscard]] std::string getUsername() const override;
  [[nodiscard]] std::string getPassword() const override;
  [[nodiscard]] double getRGBImageQuality() const override;
  [[nodiscard]] bool getHasRGBCameras() const override;
  [[nodiscard]] bool getPublishRawRGBCameras() const override;
  [[nodiscard]] bool getPublishRGBImages() const override;
  [[nodiscard]] bool getPublishDepthImages() const override;
  [[nodiscard]] bool getPublishDepthRegisteredImages() const override;
  [[nodiscard]] std::string getPreferredOdomFrame() const override;
  [[nodiscard]] std::string getSpotName() const override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace spot_ros2
