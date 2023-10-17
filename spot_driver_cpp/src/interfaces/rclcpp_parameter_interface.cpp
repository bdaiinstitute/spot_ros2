// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/interfaces/rclcpp_parameter_interface.hpp>

#include <cstdlib>

namespace {
constexpr auto kEnvVarNameAddress = "SPOT_IP";
constexpr auto kEnvVarNameUsername = "BOSDYN_CLIENT_USERNAME";
constexpr auto kEnvVarNamePassword = "BOSDYN_CLIENT_PASSWORD";

constexpr auto kParameterNameAddress = "address";
constexpr auto kParameterNameUsername = "username";
constexpr auto kParameterNamePassword = "password";
constexpr auto kParameterNameRGBImageQuality = "image_quality";
constexpr auto kParameterNameHasRGBCameras = "rgb_cameras";
constexpr auto kParameterNamePublishRGBImages = "publish_rgb";
constexpr auto kParameterNamePublishDepthImages = "publish_depth";
constexpr auto kParameterNamePublishDepthRegisteredImages = "publish_depth_registered";

/**
 * @brief Get a rclcpp parameter. If the parameter has not been declared, declare it with the provided default value and
 * then return the value.
 *
 * @tparam ParameterT Parameter type.
 * @param node Parameter will be declared and retrieved from this node.
 * @param name Name of the parameter
 * @param default_value Default value of the parameter to use if it was not already declared.
 * @return If the value for the parameter was already set, return the value. Otherwise, return the default value.
 */
template <typename ParameterT>
ParameterT declareAndGetParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name,
                                  const ParameterT& default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<ParameterT>(name, default_value);
  }

  return node->get_parameter_or<ParameterT>(name, default_value);
}

/**
 * @brief Get the value of an environment variable, if it has been set.
 *
 * @param name Name of the environment variable.
 * @return If the environment variable was set, return its value. Otherwise, return nullopt.
 */
std::optional<std::string> getEnvironmentVariable(const std::string& name) {
  const auto value = std::getenv(name.c_str());
  if (!value) {
    return std::nullopt;
  }
  return value;
}

/**
 * @brief Attempt to get a value from an environment variable, and if this fails get the value from a rclcpp parameter
 * instead.
 *
 * @param node Node to use when retrieving the rclcpp parameter.
 * @param env_var_name Name of the environment variable.
 * @param parameter_name Name of the parameter.
 * @param default_value Default value of the parameter.
 * @return If the environment variable was set, return its value. Otherwise, if the rclcpp parameter was set, return its
 * value. If neither the environment variable nor the rclcpp parameter was set. return the default value.
 */
std::string getEnvironmentVariableParameterFallback(const std::shared_ptr<rclcpp::Node>& node,
                                                    const std::string& env_var_name, const std::string& parameter_name,
                                                    const std::string& default_value) {
  if (const auto env_var_result = getEnvironmentVariable(env_var_name); env_var_result.has_value()) {
    return env_var_result.value();
  }
  return declareAndGetParameter<std::string>(node, parameter_name, default_value);
}
}  // namespace

namespace spot_ros2 {

RclcppParameterInterface::RclcppParameterInterface(const std::shared_ptr<rclcpp::Node>& node) : node_{node} {}

std::string RclcppParameterInterface::getAddress() const {
  return getEnvironmentVariableParameterFallback(node_, kEnvVarNameAddress, kParameterNameAddress, kDefaultAddress);
}

std::string RclcppParameterInterface::getUsername() const {
  return getEnvironmentVariableParameterFallback(node_, kEnvVarNameUsername, kParameterNameUsername, kDefaultUsername);
}

std::string RclcppParameterInterface::getPassword() const {
  return getEnvironmentVariableParameterFallback(node_, kEnvVarNamePassword, kParameterNamePassword, kDefaultPassword);
}

double RclcppParameterInterface::getRGBImageQuality() const {
  return declareAndGetParameter<double>(node_, kParameterNameRGBImageQuality, kDefaultRGBImageQuality);
}

bool RclcppParameterInterface::getHasRGBCameras() const {
  return declareAndGetParameter<bool>(node_, kParameterNameHasRGBCameras, kDefaultHasRGBCameras);
}

bool RclcppParameterInterface::getPublishRGBImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishRGBImages, kDefaultPublishRGBImages);
}

bool RclcppParameterInterface::getPublishDepthImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishDepthImages, kDefaultPublishDepthImages);
}

bool RclcppParameterInterface::getPublishDepthRegisteredImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishDepthRegisteredImages,
                                      kDefaultPublishDepthRegisteredImages);
}

std::string RclcppParameterInterface::getSpotName() const {
  // The spot_name parameter always matches the namespace of this node, minus the leading `/` character.
  try {
    return std::string{node_->get_namespace()}.substr(1);
  } catch (const std::out_of_range& e) {
    // get_namespace() should not return an empty string, but we handle this situation just in case.
    // Note that if no namespace was set when creating the node, get_namespace() will return `/`.
    return "";
  }
}
}  // namespace spot_ros2
