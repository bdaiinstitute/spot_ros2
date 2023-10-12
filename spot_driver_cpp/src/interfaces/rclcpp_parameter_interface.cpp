// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>

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
}
