// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_publisher.hpp>

#include <rclcpp/node.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_publisher_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_tf_interface.hpp>
#include <spot_driver_cpp/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver_cpp/interfaces/spot_interface.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/types.hpp>

#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>

namespace
{
constexpr auto kImageCallbackPeriod = std::chrono::duration<double>{1.0 / 15.0 };  // 15 Hz
constexpr auto kDefaultDepthImageQuality = 100.0;
}  // namespace

namespace spot_ros2
{
::bosdyn::api::GetImageRequest createImageRequest(const std::vector<ImageSource>& sources, [[maybe_unused]] const bool has_rgb_cameras, const double rgb_image_quality, const bool get_raw_rgb_images)
{
    ::bosdyn::api::GetImageRequest request_message;

    for (const auto& source : sources)
    {
        const auto source_name = toSpotImageSourceName(source);
        if (source.type == SpotImageType::RGB)
        {
            bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
            image_request->set_image_source_name(source_name);
            // RGB images can have a user-configurable image quality setting.
            image_request->set_quality_percent(rgb_image_quality);
            image_request->set_pixel_format(bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8);
            // RGB images can be either raw or JPEG-compressed.
            image_request->set_image_format(get_raw_rgb_images ? bosdyn::api::Image_Format_FORMAT_RAW : bosdyn::api::Image_Format_FORMAT_JPEG);
        }
        else if (source.type == SpotImageType::DEPTH)
        {
            bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
            image_request->set_image_source_name(source_name);
            image_request->set_quality_percent(kDefaultDepthImageQuality);
            image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
        }
        else // SpotImageType::DEPTH_REGISTERED
        {
            bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
            image_request->set_image_source_name(source_name);
            image_request->set_quality_percent(kDefaultDepthImageQuality);
            image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
        }
    }  

    return request_message;
}

SpotImagePublisher::SpotImagePublisher(std::unique_ptr<TimerInterfaceBase> timer_interface, std::unique_ptr<SpotInterfaceBase> spot_interface, std::unique_ptr<PublisherInterfaceBase> publisher_interface, std::unique_ptr<ParameterInterfaceBase> parameter_interface, std::unique_ptr<TfInterfaceBase> tf_interface)
: timer_interface_{ std::move(timer_interface) }
, spot_interface_{ std::move(spot_interface) }
, publisher_interface_{ std::move(publisher_interface) }
, parameter_interface_{ std::move(parameter_interface) }
, tf_interface_{ std::move(tf_interface) }
{
}

SpotImagePublisher::SpotImagePublisher(const std::shared_ptr<rclcpp::Node>& node)
: SpotImagePublisher( std::make_unique<RclcppWallTimerInterface>(node), std::make_unique<SpotInterface>(), std::make_unique<RclcppPublisherInterface>(node), std::make_unique<RclcppParameterInterface>(node), std::make_unique<RclcppTfInterface>(node))
{
}

bool SpotImagePublisher::initialize()
{
    // These parameters all fall back to default values if the user did not set them at runtime
    const auto address = parameter_interface_->getAddress();
    const auto username = parameter_interface_->getUsername();
    const auto password = parameter_interface_->getPassword();
    const auto rgb_image_quality = parameter_interface_->getRGBImageQuality();
    const auto publish_rgb_images = parameter_interface_->getPublishRGBImages();
    const auto publish_depth_images = parameter_interface_->getPublishDepthImages();
    const auto publish_depth_registered_images = parameter_interface_->getPublishDepthRegisteredImages();
    const auto has_rgb_cameras = parameter_interface_->getHasRGBCameras();
    const auto spot_name = parameter_interface_->getSpotName();

    // Initialize the SDK client, and connect to the robot
    if (const auto result = spot_interface_->createRobot(address, spot_name); !result)
    {
        std::cerr << "Failed to create interface to robot: " << result.error() << std::endl;
        return false;
    }

    if (const auto result = spot_interface_->authenticate(username, password); !result)
    {
        std::cerr << "Failed to authenticate with robot: " << result.error() << std::endl;
        return false;
    }

    const auto has_arm_result = spot_interface_->hasArm();
    if (!has_arm_result)
    {
        std::cerr << "Failed to determine if Spot is equipped with an arm: " << has_arm_result.error() << std::endl;
        return false;
    }

    // Generate the list of image sources based on which cameras the user has requested that we publish
    const auto sources = createImageSourcesList(publish_rgb_images, publish_depth_images, publish_depth_registered_images, has_arm_result.value());

    // Generate the image request message to capture the data from the specified image sources
    image_request_message_ = createImageRequest(sources, has_rgb_cameras, rgb_image_quality, false);

    // Create a publisher for each image source
    publisher_interface_->createPublishers(sources);

    // Create a timer to request and publish images at a fixed rate
    timer_interface_->setTimer(kImageCallbackPeriod, [this](){ timerCallback(); });

    return true;
}

void SpotImagePublisher::timerCallback()
{
    if (!image_request_message_)
    {
        return;
    }

    const auto image_result = spot_interface_->getImages(*image_request_message_);
    if(!image_result.has_value())
    {
        std::cerr << "Failed to get images: " << image_result.error() << std::endl;
        return;
    }

    publisher_interface_->publish(image_result.value().images_);

    tf_interface_->publishStaticTransforms(image_result.value().transforms_);
}
} // namespace spot_ros2
