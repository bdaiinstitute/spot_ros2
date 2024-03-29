// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/rclcpp_test.hpp>

#include <memory>

namespace {
using ::testing::Eq;
using ::testing::IsEmpty;
using ::testing::IsTrue;
using ::testing::StrEq;

constexpr auto kNodeName = "my_node_name";
constexpr auto kNamespace = "my_namespace";

constexpr auto kEnvVarNameHostname = "SPOT_IP";
constexpr auto kEnvVarNameUsername = "BOSDYN_CLIENT_USERNAME";
constexpr auto kEnvVarNamePassword = "BOSDYN_CLIENT_PASSWORD";
}  // namespace

namespace spot_ros2::test {
class RclcppParameterInterfaceTest : public RclcppTest {};

/**
 * @brief Test fixture that caches any previously-set environment variables which would be overwritten by the test case
 * and restores them after the test is finished.
 * @details This is to prevent interfering with the user's environment if they are running the Spot driver nodes in the
 * same environment as these tests.
 */
class RclcppParameterInterfaceEnvVarTest : public RclcppParameterInterfaceTest {
 public:
  void SetUp() override {
    RclcppTest::SetUp();

    // Get current values of these environment variables.
    const auto hostname = std::getenv(kEnvVarNameHostname);
    const auto username = std::getenv(kEnvVarNameUsername);
    const auto password = std::getenv(kEnvVarNamePassword);

    // If any are already set, cache them in private members.
    if (hostname) {
      spot_hostname_env_var_cached_ = hostname;
    }
    if (username) {
      spot_username_env_var_cached_ = username;
    }
    if (password) {
      spot_password_env_var_cached_ = password;
    }

    // Unset the values of the environment variables to create a clean environment for the test cases.
    unsetenv(kEnvVarNameHostname);
    unsetenv(kEnvVarNameUsername);
    unsetenv(kEnvVarNamePassword);

    // All test cases that use this fixture also use a node, so create a default node here too.
    node_ = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  }

  void TearDown() override {
    RclcppTest::TearDown();

    // Restore any cached environment variables.
    if (spot_hostname_env_var_cached_) {
      // Calling setenv with the last parameter == 1 replaces any existing value of the environment variable with the
      // new value.
      setenv(kEnvVarNameHostname, spot_hostname_env_var_cached_->c_str(), 1);
    }
    if (spot_username_env_var_cached_) {
      setenv(kEnvVarNameUsername, spot_username_env_var_cached_->c_str(), 1);
    }
    if (spot_password_env_var_cached_) {
      setenv(kEnvVarNamePassword, spot_password_env_var_cached_->c_str(), 1);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;

 private:
  std::optional<std::string> spot_hostname_env_var_cached_;
  std::optional<std::string> spot_username_env_var_cached_;
  std::optional<std::string> spot_password_env_var_cached_;
};

TEST_F(RclcppParameterInterfaceTest, GetSpotNameWithNamespace) {
  // GIVEN we create a rclcpp node with a namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we call getSpotName
  // THEN the parameter interface returns the namespace of the node
  EXPECT_THAT(parameter_interface.getSpotName(), StrEq(kNamespace));
}

TEST_F(RclcppParameterInterfaceTest, GetSpotNameWithEmptyNamespace) {
  // GIVEN we create a rclcpp node with an empty namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName, "");
  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we call getSpotName
  // THEN the parameter interface returns an empty string
  EXPECT_THAT(parameter_interface.getSpotName(), IsEmpty());
}

TEST_F(RclcppParameterInterfaceTest, GetSpotNameWithDefaultNamespace) {
  // GIVEN we create a rclcpp node and do not set a specific namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName);
  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we call getSpotName
  // THEN the parameter interface returns an empty string
  EXPECT_THAT(parameter_interface.getSpotName(), IsEmpty());
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSpotConfigFromEnvVars) {
  // GIVEN we declare environment variables for Spot's hostname, username, and password
  constexpr auto hostname_env_var = "10.0.20.5";
  setenv(kEnvVarNameHostname, hostname_env_var, 1);
  constexpr auto username_env_var = "some_username";
  setenv(kEnvVarNameUsername, username_env_var, 1);
  constexpr auto password_env_var = "very_secure_password";
  setenv(kEnvVarNamePassword, password_env_var, 1);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we call getHostname(), getUsername(), and getPassword()
  // THEN the returned values match the values we set on the environment variables
  EXPECT_THAT(parameter_interface.getHostname(), StrEq(hostname_env_var));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq(username_env_var));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq(password_env_var));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSpotConfigFromParameters) {
  // GIVEN we set all Spot config parameters to values which are different than the default values
  constexpr auto hostname_parameter = "192.168.100.10";
  node_->declare_parameter("hostname", hostname_parameter);
  constexpr auto username_parameter = "a_different_username";
  node_->declare_parameter("username", username_parameter);
  constexpr auto password_parameter = "other_password";
  node_->declare_parameter("password", password_parameter);
  constexpr auto rgb_image_quality_parameter = 42.0;
  node_->declare_parameter("image_quality", rgb_image_quality_parameter);
  constexpr auto has_rgb_cameras_parameter = false;
  node_->declare_parameter("rgb_cameras", has_rgb_cameras_parameter);
  constexpr auto publish_rgb_images_parameter = false;
  node_->declare_parameter("publish_rgb", publish_rgb_images_parameter);
  constexpr auto publish_depth_images_parameter = false;
  node_->declare_parameter("publish_depth", publish_depth_images_parameter);
  constexpr auto publish_depth_registered_images_parameter = false;
  node_->declare_parameter("publish_depth_registered", publish_depth_registered_images_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN the returned values all match the values we used when declaring the parameters
  EXPECT_THAT(parameter_interface.getHostname(), StrEq(hostname_parameter));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq(username_parameter));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq(password_parameter));
  EXPECT_THAT(parameter_interface.getRGBImageQuality(), Eq(rgb_image_quality_parameter));
  EXPECT_THAT(parameter_interface.getHasRGBCameras(), Eq(has_rgb_cameras_parameter));
  EXPECT_THAT(parameter_interface.getPublishRGBImages(), Eq(publish_rgb_images_parameter));
  EXPECT_THAT(parameter_interface.getPublishDepthImages(), Eq(publish_depth_images_parameter));
  EXPECT_THAT(parameter_interface.getPublishDepthRegisteredImages(), Eq(publish_depth_registered_images_parameter));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSpotConfigEnvVarsOverruleParameters) {
  // GIVEN we declare a environment variables for Spot's hostname, username, and password
  constexpr auto hostname_env_var = "10.0.20.5";
  setenv(kEnvVarNameHostname, hostname_env_var, 1);
  constexpr auto username_env_var = "some_username";
  setenv(kEnvVarNameUsername, username_env_var, 1);
  constexpr auto password_env_var = "very_secure_password";
  setenv(kEnvVarNamePassword, password_env_var, 1);

  // GIVEN we set parameters with different values for Spot's hostname, username, and password
  constexpr auto hostname_parameter = "192.168.100.10";
  node_->declare_parameter("hostname", hostname_parameter);
  constexpr auto username_parameter = "a_different_username";
  node_->declare_parameter("username", username_parameter);
  constexpr auto password_parameter = "other_password";
  node_->declare_parameter("password", password_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we call getHostname(), getUsername(), and getPassword()
  // THEN the returned values match the values we used when declaring the environment variables, since we expect that
  // the environment variables take precedence over the parameters.
  EXPECT_THAT(parameter_interface.getHostname(), StrEq(hostname_env_var));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq(username_env_var));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq(password_env_var));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetConfigDefaults) {
  // GIVEN we do not set any environment variables or parameters
  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we get the values from the parameter interface
  // THEN the returned values match the expected default values
  EXPECT_THAT(parameter_interface.getHostname(), StrEq("10.0.0.3"));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq("user"));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq("password"));
  EXPECT_THAT(parameter_interface.getRGBImageQuality(), Eq(70.0));
  EXPECT_THAT(parameter_interface.getHasRGBCameras(), IsTrue());
  EXPECT_THAT(parameter_interface.getPublishRGBImages(), IsTrue());
  EXPECT_THAT(parameter_interface.getPublishDepthImages(), IsTrue());
  EXPECT_THAT(parameter_interface.getPublishDepthRegisteredImages(), IsTrue());
}
}  // namespace spot_ros2::test
