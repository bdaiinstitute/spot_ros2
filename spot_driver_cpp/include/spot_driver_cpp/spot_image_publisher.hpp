// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver_cpp/spot_interface.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace spot_ros2
{
struct ImageSources
{
  std::vector<std::string> rgb;
  std::vector<std::string> depth;
  std::vector<std::string> depth_registered;
};

enum class SpotImageType
{
  RGB,
  DEPTH,
};

class TimerInterfaceBase
{
public:
  virtual ~TimerInterfaceBase() {};

  virtual void setTimer(const std::chrono::duration<double>& period, const std::function<void()>& callback) = 0;

  virtual void clearTimer() = 0;
};

class RclcppWallTimerInterface : public TimerInterfaceBase
{
public:
  RclcppWallTimerInterface(const std::shared_ptr<rclcpp::Node>& node);

  void setTimer(const std::chrono::duration<double>& period, const std::function<void()>& callback) override;

  void clearTimer() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

class PublisherInterfaceBase
{
public:
  virtual ~PublisherInterfaceBase() {};

  virtual void createPublishers(const ImageSources& image_sources) = 0;

  virtual void publishImages(const std::unordered_map<std::string, sensor_msgs::msg::Image>& images) = 0;
};

class RclcppPublisherInterface : public PublisherInterfaceBase
{
public:
  RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node);

  void createPublishers(const ImageSources& image_sources) override;

  void publishImages(const std::unordered_map<std::string, sensor_msgs::msg::Image>& images) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> publishers_;
};

class ParameterInterfaceBase
{
public:
  virtual ~ParameterInterfaceBase() {};

  virtual std::optional<std::string> getAddress() const = 0;
  virtual std::optional<std::string> getUsername() const = 0;
  virtual std::optional<std::string> getPassword() const = 0;

  virtual double getRGBImageQuality() const = 0;
  virtual bool getHasRGBCameras() const = 0;
  virtual bool getPublishRGBImages() const = 0;
  virtual bool getPublishDepthImages() const = 0;
  virtual bool getPublishDepthRegisteredImages() const = 0;

  static constexpr double kDefaultRGBImageQuality{ 70.0 };
  static constexpr bool kDefaultHasRGBCameras{ true };
  static constexpr bool kDefaultPublishRGBImages{ true };
  static constexpr bool kDefaultPublishDepthImages{ true };
  static constexpr bool kDefaultPublishDepthRegisteredImages{ true };
};

class RclcppParameterInterface : public ParameterInterfaceBase
{
public:
  RclcppParameterInterface(const std::shared_ptr<rclcpp::Node>& node);
  std::optional<std::string> getAddress() const override;
  std::optional<std::string> getUsername() const override;
  std::optional<std::string> getPassword() const override;
  double getRGBImageQuality() const override;
  bool getHasRGBCameras() const override;
  bool getPublishRGBImages() const override;
  bool getPublishDepthImages() const override;
  bool getPublishDepthRegisteredImages() const override;
private:
  std::shared_ptr<rclcpp::Node> node_;
};

std::string createTopicName(const std::string& camera_name, const SpotImageType& image_type);

ImageSources createImageSourcesList(const bool get_rgb_images, const bool get_depth_images, const bool get_depth_registered_images, const bool has_hand_camera);

::bosdyn::api::GetImageRequest createImageRequest(const ImageSources& sources, const bool has_rgb_cameras, const double rgb_image_quality, const bool get_raw_rgb_images);

class SpotImagePublisher
{
public:
  SpotImagePublisher(std::unique_ptr<TimerInterfaceBase> timer_interface, std::unique_ptr<SpotInterfaceBase> spot_interface, std::unique_ptr<PublisherInterfaceBase> publisher_interface, std::unique_ptr<ParameterInterfaceBase> parameter_interface);

  SpotImagePublisher(const std::shared_ptr<rclcpp::Node>& node);

  bool initialize();

private:
  void timerCallback();

  std::optional<::bosdyn::api::GetImageRequest> image_request_message_;

  std::unique_ptr<TimerInterfaceBase> timer_interface_;
  std::unique_ptr<SpotInterfaceBase> spot_interface_;
  std::unique_ptr<PublisherInterfaceBase> publisher_interface_;
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
};

class SpotImagePublisherNode
{
public:
  SpotImagePublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

private:
  std::shared_ptr<rclcpp::Node> node_;
  SpotImagePublisher internal_;
};
}
