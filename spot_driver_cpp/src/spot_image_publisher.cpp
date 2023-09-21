// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_publisher.hpp>

#include <rclcpp/node.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
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

constexpr auto kParameterNameAddress = "address";
constexpr auto kParameterNameUsername = "username";
constexpr auto kParameterNamePassword = "password";
constexpr auto kParameterNameRGBImageQuality = "image_quality";
constexpr auto kParameterNameHasRGBCameras = "rgb_cameras";
constexpr auto kParameterNamePublishRGBImages = "publish_rgb";
constexpr auto kParameterNamePublishDepthImages = "publish_depth";
constexpr auto kParameterNamePublishDepthRegisteredImages = "publish_depth_registered";

template<typename ParameterT>
std::optional<ParameterT> declareAndGetParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name)
{
    if (!node->has_parameter(name))
    {
        node->declare_parameter<ParameterT>(name);
    }

    ParameterT out;
    if(!node->get_parameter<ParameterT>(name, out))
    {
        return std::nullopt;
    }
    return out;
}

template<typename ParameterT>
ParameterT declareAndGetParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, const ParameterT& default_value)
{
    if (!node->has_parameter(name))
    {
        node->declare_parameter<ParameterT>(name);
    }

    return node->get_parameter_or<ParameterT>(name, default_value);
}
}  // namespace

namespace spot_ros2
{
RclcppWallTimerInterface::RclcppWallTimerInterface(const std::shared_ptr<rclcpp::Node>& node)
: node_{node}
{
}

void RclcppWallTimerInterface::setTimer(const std::chrono::duration<double>& period, const std::function<void()>& callback)
{
    timer_ = node_->create_wall_timer(period, callback);
}

void RclcppWallTimerInterface::clearTimer()
{
    timer_.reset();
}

RclcppPublisherInterface::RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node)
: node_{ node }
{
}

void RclcppPublisherInterface::createPublishers(const std::vector<ImageSource>& image_sources)
{
    image_publishers_.clear();
    info_publishers_.clear();

    for (const auto& image_source : image_sources)
    {
        // Since these topic names do not have a leading `/` character, they will be published within the namespace of the node, which should match the name of the robot.
        // For example, the topic for the front left RGB camera will ultimately appear as `/MyRobotName/camera/frontleft/image`.
        const auto topic_name_base = toRosTopic(image_source);

        const auto image_topic_name = topic_name_base + "/image";
        image_publishers_.try_emplace(image_topic_name, node_->create_publisher<sensor_msgs::msg::Image>(image_topic_name, rclcpp::QoS(1)));

        const auto info_topic_name = topic_name_base + "/camera_info";
        info_publishers_.try_emplace(info_topic_name, node_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name, rclcpp::QoS(1)));
    }
}

void RclcppPublisherInterface::publish(const std::map<ImageSource, ImageWithCameraInfo>& images)
{
    for (const auto& [image_source, image_data] : images)
    {
        const auto topic_name_base = toRosTopic(image_source);
        const auto image_topic_name = topic_name_base + "/image";
        const auto info_topic_name = topic_name_base + "/camera_info";

        try
        {
            image_publishers_.at(image_topic_name)->publish(image_data.image);
            info_publishers_.at(info_topic_name)->publish(image_data.info);
        }
        catch(const std::out_of_range& e)
        {
            std::cerr << "No publisher exists for image source " << image_source.name << std::endl;
        }
    }
}

RclcppParameterInterface::RclcppParameterInterface(const std::shared_ptr<rclcpp::Node>& node)
: node_{node}
{
}

std::optional<std::string> RclcppParameterInterface::getAddress() const
{
    return declareAndGetParameter<std::string>(node_, kParameterNameAddress);
}

std::optional<std::string> RclcppParameterInterface::getUsername() const
{
    return declareAndGetParameter<std::string>(node_, kParameterNameUsername);
}

std::optional<std::string> RclcppParameterInterface::getPassword() const
{
    return declareAndGetParameter<std::string>(node_, kParameterNamePassword);
}

double RclcppParameterInterface::getRGBImageQuality() const
{
    return declareAndGetParameter<double>(node_, kParameterNameRGBImageQuality, kDefaultRGBImageQuality);
}

bool RclcppParameterInterface::getHasRGBCameras() const
{
    return declareAndGetParameter<bool>(node_, kParameterNameHasRGBCameras, kDefaultHasRGBCameras);
}

bool RclcppParameterInterface::getPublishRGBImages() const
{
    return declareAndGetParameter<bool>(node_, kParameterNamePublishRGBImages, kDefaultPublishRGBImages);
}

bool RclcppParameterInterface::getPublishDepthImages() const
{
    return declareAndGetParameter<bool>(node_, kParameterNamePublishDepthImages, kDefaultPublishDepthImages);
}

bool RclcppParameterInterface::getPublishDepthRegisteredImages() const
{
    return declareAndGetParameter<bool>(node_, kParameterNamePublishDepthRegisteredImages, kDefaultPublishDepthRegisteredImages);
}

std::string RclcppParameterInterface::getSpotName() const
{
    // The spot_name parameter always matches the namespace of this node, minus the leading `/` character.
    try
    {
        return std::string{node_->get_namespace()}.substr(1);
    }
    catch(const std::out_of_range& e)
    {
        // get_namespace() should not return an empty string, but we handle this situation just in case.
        return "";
    }
}

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
            image_request->set_quality_percent(rgb_image_quality);
            image_request->set_pixel_format(bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8);
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

SpotImagePublisher::SpotImagePublisher(std::unique_ptr<TimerInterfaceBase> timer_interface, std::unique_ptr<SpotInterfaceBase> spot_interface, std::unique_ptr<PublisherInterfaceBase> publisher_interface, std::unique_ptr<ParameterInterfaceBase> parameter_interface)
: timer_interface_{ std::move(timer_interface) }
, spot_interface_{ std::move(spot_interface) }
, publisher_interface_{ std::move(publisher_interface) }
, parameter_interface_{ std::move(parameter_interface) }
{
}

SpotImagePublisher::SpotImagePublisher(const std::shared_ptr<rclcpp::Node>& node)
: SpotImagePublisher( std::make_unique<RclcppWallTimerInterface>(node), std::make_unique<SpotInterface>(), std::make_unique<RclcppPublisherInterface>(node), std::make_unique<RclcppParameterInterface>(node))
{
}

bool SpotImagePublisher::initialize()
{
    // Get and validate parameters which do not have default values, and which are required to be set at runtime
    const auto address = parameter_interface_->getAddress();
    const auto username = parameter_interface_->getUsername();
    const auto password = parameter_interface_->getPassword();

    if (!address.has_value())
    {
        std::cerr << "Failed to get address" << std::endl;
        return false;
    }

    if (!username.has_value())
    {
        std::cerr << "Failed to get username" << std::endl;
        return false;
    }

    if (!password.has_value())
    {
        std::cerr << "Failed to get password" << std::endl;
        return false;
    }

    // Get parameters which fall back to default values if the user did not set them at runtime
    const auto rgb_image_quality = parameter_interface_->getRGBImageQuality();
    const auto publish_rgb_images = parameter_interface_->getPublishRGBImages();
    const auto publish_depth_images = parameter_interface_->getPublishDepthImages();
    const auto publish_depth_registered_images = parameter_interface_->getPublishDepthRegisteredImages();
    const auto has_rgb_cameras = parameter_interface_->getHasRGBCameras();
    const auto spot_name = parameter_interface_->getSpotName();

    // Initialize the SDK client, and connect to the robot
    if (const auto result = spot_interface_->createRobot(*address, spot_name); !result)
    {
        std::cerr << "Failed to create interface to robot: " << result.error() << std::endl;
        return false;
    }

    if (const auto result = spot_interface_->authenticate(*username, *password); !result)
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

    const auto images = spot_interface_->getImages(*image_request_message_);
    if(!images.has_value())
    {
        std::cerr << "Failed to get images: " << images.error() << std::endl;
        return;
    }

    publisher_interface_->publish(images.value());
}


SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options)
: node_{ std::make_shared<rclcpp::Node>( "image_publisher" , node_options) }
, internal_{ SpotImagePublisher{ node_ } }
{
    internal_.initialize();
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface()
{
    return node_->get_node_base_interface();
}
}
