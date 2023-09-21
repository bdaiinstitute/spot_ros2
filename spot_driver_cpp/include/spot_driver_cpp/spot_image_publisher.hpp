// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/spot_interface.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace spot_ros2
{
/**
 * @brief Defines an interface for a class that calls a callback function at a specified rate.
 */
class TimerInterfaceBase
{
public:
  virtual ~TimerInterfaceBase() {};

  virtual void setTimer(const std::chrono::duration<double>& period, const std::function<void()>& callback) = 0;
  virtual void clearTimer() = 0;
};

/**
 * @brief Implements TimerInterfaceBase to use rclcpp's WallTimer.
 */
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

/**
 * @brief Defines an interface for a class that publishes image data to middleware.
 */
class PublisherInterfaceBase
{
public:
  virtual ~PublisherInterfaceBase() {};

  virtual void createPublishers(const std::vector<ImageSource>& image_sources) = 0;
  virtual void publishImages(const std::map<ImageSource, sensor_msgs::msg::Image>& images) = 0;
};

/**
 * @brief Implements PublisherInterfaceBase to use rclcpp Publishers.
 */
class RclcppPublisherInterface : public PublisherInterfaceBase
{
public:
  RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node);

  void createPublishers(const std::vector<ImageSource>& image_sources) override;
  void publishImages(const std::map<ImageSource, sensor_msgs::msg::Image>& images) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> publishers_;
};

/**
 * @brief Defines an interface for a class that retrieves the user-configured parameters needed to connect to Spot.
 */
class ParameterInterfaceBase
{
public:
  virtual ~ParameterInterfaceBase() {};

  // These functions retrieve required parameters, where it is not possible to connect to Spot unless each parameter is set by the user to a specific value.
  // If the parameter was set, return its value.
  // If the parameter was not set, return std::nullopt.
  virtual std::optional<std::string> getAddress() const = 0;
  virtual std::optional<std::string> getUsername() const = 0;
  virtual std::optional<std::string> getPassword() const = 0;

  // These functions retrieve optional parameters, where a default value can be used if the user does not provide a specific value.
  // If the parameter was set, return the value provided by the user.
  // If the parameter was not set, return the default value.
  virtual double getRGBImageQuality() const = 0;
  virtual bool getHasRGBCameras() const = 0;
  virtual bool getPublishRGBImages() const = 0;
  virtual bool getPublishDepthImages() const = 0;
  virtual bool getPublishDepthRegisteredImages() const = 0;

protected:
  // These are the definitions of the default values for optional parameters.
  static constexpr double kDefaultRGBImageQuality{ 70.0 };
  static constexpr bool kDefaultHasRGBCameras{ true };
  static constexpr bool kDefaultPublishRGBImages{ true };
  static constexpr bool kDefaultPublishDepthImages{ true };
  static constexpr bool kDefaultPublishDepthRegisteredImages{ true };
};

/**
 * @brief Implements ParameterInterfaceBase to declare and retrieve ROS 2 parameters.
 */
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

/**
 * @brief Create a Spot API GetImageRequest message to request images from the specified sources using the specified options.
 * @details Requests for depth images will always use FORMAT_RAW at quality 100.0.
 * 
 * @param sources List of image sources. Defines which cameras to request images from.
 * @param has_rgb_cameras Set this to true if Spot's 2D body cameras can capture RGB image data.
 * @param rgb_image_quality A value in the range from 0.0 to 100.0 which defines the level of compression to use when requesting JPEG-compressed RGB image data.
 * @param get_raw_rgb_images If true, request raw images from Spot's 2D body cameras. If false, request JPEG-compressed images.
 * @return ::bosdyn::api::GetImageRequest A GetImageRequest message equivalent to the input parameters.
 */
::bosdyn::api::GetImageRequest createImageRequest(const std::vector<ImageSource>& sources, const bool has_rgb_cameras, const double rgb_image_quality, const bool get_raw_rgb_images);

/**
 * @brief A class to connect to and authenticate with Spot, retrieve images from its cameras, and publish each image to a ROS topic.
 */
class SpotImagePublisher
{
public:
  SpotImagePublisher(std::unique_ptr<TimerInterfaceBase> timer_interface, std::unique_ptr<SpotInterfaceBase> spot_interface, std::unique_ptr<PublisherInterfaceBase> publisher_interface, std::unique_ptr<ParameterInterfaceBase> parameter_interface);

  SpotImagePublisher(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Connect to Spot and start publishing image data.
   * 
   * @return True, if connection and authentication succeeded.
   * @return False, if a required user-defined parameter was not set, if the attempt to connect to Spot fails, or if the attempt to authenticate with Spot using the provided credentials fails.
   */
  bool initialize();

private:
  void timerCallback();

  std::optional<::bosdyn::api::GetImageRequest> image_request_message_;

  std::unique_ptr<TimerInterfaceBase> timer_interface_;
  std::unique_ptr<SpotInterfaceBase> spot_interface_;
  std::unique_ptr<PublisherInterfaceBase> publisher_interface_;
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;
};

/**
 * @brief Wraps SpotImagePublisher to allow using it like a rclcpp::Node.
 */
class SpotImagePublisherNode
{
public:
  SpotImagePublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node. This allows loading this class as a component node in a composable node container.
   * 
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

private:
  std::shared_ptr<rclcpp::Node> node_;
  SpotImagePublisher internal_;
};
}
