// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/spot_image_publisher.hpp>

#include <rclcpp/node.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>

#include <iostream>
#include <memory>
#include <optional>

namespace
{
constexpr auto kImageCallbackPeriod = std::chrono::duration<double>{1.0 / 15.0 };
constexpr auto kDefaultDepthImageQuality = 100.0;

constexpr auto kParameterNameAddress = "address";
constexpr auto kParameterNameUsername = "username";
constexpr auto kParameterNamePassword = "password";
constexpr auto kParameterNameRGBImageQuality = "image_quality";
constexpr auto kParameterNameHasRGBCameras = "rgb_cameras";
constexpr auto kParameterNamePublishRGBImages = "publish_rgb";
constexpr auto kParameterNamePublishDepthImages = "publish_depth";
constexpr auto kParameterNamePublishDepthRegisteredImages = "publish_depth_registered";

const std::vector<std::string> kImageSourceNamesRGB = {
    "back_fisheye_image",
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
};

const std::vector<std::string> kImageSourceNamesDepth = {
    "back_depth",
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
};

const std::vector<std::string> kImageSourceNamesDepthRegistered = {
    "back_depth_registered",
    "frontleft_depth_registered",
    "frontright_depth_registered",
    "left_depth_registered",
    "right_depth_registered",
};

constexpr auto kImageSourceHandRGB = "hand_color_image";
constexpr auto kImageSourceHandDepth = "hand_depth";
constexpr auto kImageSourceHandDepthRegistered = "hand_depth_registered";

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

void RclcppPublisherInterface::createPublishers(const ImageSources& image_sources)
{
    publishers_.clear();

    std::vector<std::string> image_sources_combined;
    image_sources_combined.insert(image_sources_combined.end(), image_sources.rgb.cbegin(), image_sources.rgb.cend());
    image_sources_combined.insert(image_sources_combined.end(), image_sources.depth.cbegin(), image_sources.depth.cend());
    image_sources_combined.insert(image_sources_combined.end(), image_sources.depth_registered.cbegin(), image_sources.depth_registered.cend());

    for (const auto& image_source : image_sources_combined)
    {
        publishers_.try_emplace(image_source, node_->create_publisher<sensor_msgs::msg::Image>(image_source, rclcpp::QoS(1)));
    }
}

void RclcppPublisherInterface::publishImages(const std::unordered_map<std::string, sensor_msgs::msg::Image>& images)
{
    for (const auto& [name, image_data] : images)
    {
        std::cout << name << std::endl;
        const auto publisher = publishers_.at(name);
        publisher->publish(image_data);
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

std::string createTopicName(const std::string& camera_name, const SpotImageType& image_type)
{
  if (image_type == SpotImageType::RGB)
  {
    return std::string("camera/").append(camera_name).append("/image");
  }
  else
  {
    return std::string("depth/").append(camera_name).append("/image");
  }
}

ImageSources createImageSourcesList(const bool get_rgb_images, const bool get_depth_images, const bool get_depth_registered_images, const bool has_hand_camera)
{
    ImageSources sources;
    if (get_rgb_images)
    {
        sources.rgb = kImageSourceNamesRGB;
        if (has_hand_camera)
        {
            sources.rgb.push_back(kImageSourceHandRGB);
        }
    }

    if (get_depth_images)
    {
        sources.depth = kImageSourceNamesDepth;
        if (has_hand_camera)
        {
            sources.depth.push_back(kImageSourceHandDepth);
        }
    }

    if (get_depth_registered_images)
    {
        sources.depth_registered = kImageSourceNamesDepthRegistered;
        if (has_hand_camera)
        {
            sources.depth_registered.push_back(kImageSourceHandDepthRegistered);
        }
    }

    return sources;
}

::bosdyn::api::GetImageRequest createImageRequest(const ImageSources& sources, [[maybe_unused]] const bool has_rgb_cameras, const double rgb_image_quality, const bool get_raw_rgb_images)
{
    ::bosdyn::api::GetImageRequest request_message;
    for (const auto& source : sources.rgb)
    {
        // TODO: if Spot has grayscale body cameras instead of RGB, request grayscale images
        bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
        image_request->set_image_source_name(source);
        image_request->set_quality_percent(rgb_image_quality);
        image_request->set_pixel_format(bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8);
        image_request->set_image_format(get_raw_rgb_images ? bosdyn::api::Image_Format_FORMAT_RAW : bosdyn::api::Image_Format_FORMAT_JPEG);
    }

    for (const auto& source : sources.depth)
    {
        bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
        image_request->set_image_source_name(source);
        image_request->set_quality_percent(kDefaultDepthImageQuality);
        image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
    }

    for (const auto& source : sources.depth_registered)
    {
        bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
        image_request->set_image_source_name(source);
        image_request->set_quality_percent(kDefaultDepthImageQuality);
        image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
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

    // Initialize the SDK client, and connect to the robot
    if (!spot_interface_->createRobot(*address))
    {
        std::cerr << "Failed to create robot at IP address" << std::endl;
        return false;
    }

    if (!spot_interface_->authenticate(*username, *password))
    {
        std::cerr << "Failed to authenticate with robot" << std::endl;
        return false;
    }

    // Generate the list of image sources based on which cameras the user has requested that we publish
    const auto sources = createImageSourcesList(publish_rgb_images, publish_depth_images, publish_depth_registered_images, spot_interface_->hasArm());

    // Generate the image request message to capture the data from the specified image sources
    image_request_message_ = createImageRequest(sources, has_rgb_cameras, rgb_image_quality, false);

    // Create a publisher for each image source
    publisher_interface_->createPublishers(sources);

    // Create a timer to request and publish images at a fixed rate
    timer_interface_->setTimer(kImageCallbackPeriod, [this](){ timerCallback(); });

    std::cout << "connected to robot!" << std::endl;

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
        return;
    }

    publisher_interface_->publishImages(images.value());
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
