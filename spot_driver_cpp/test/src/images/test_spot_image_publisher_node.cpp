// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/api/spot_api.hpp>
#include <spot_driver_cpp/images/spot_image_publisher_node.hpp>

#include <exception>
#include <memory>
#include <optional>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace spot_ros2::images::testing {

class MockImageClientApi : public ImageClientApi {
 public:
  MockImageClientApi() = default;
  MOCK_METHOD((tl::expected<GetImagesResult, std::string>), getImages, (::bosdyn::api::GetImageRequest request),
              (override));
};

class MockLoggerInterface : public LoggerInterfaceBase {
 public:
  MOCK_METHOD(void, logDebug, (const std::string& message), (const, override));
  MOCK_METHOD(void, logInfo, (const std::string& message), (const, override));
  MOCK_METHOD(void, logWarn, (const std::string& message), (const, override));
  MOCK_METHOD(void, logError, (const std::string& message), (const, override));
  MOCK_METHOD(void, logFatal, (const std::string& message), (const, override));
};

class FakeParameterInterface : public ParameterInterfaceBase {
 public:
  std::string getAddress() const override { return address; }

  std::string getUsername() const override { return username; }

  std::string getPassword() const override { return password; }

  double getRGBImageQuality() const override { return rgb_image_quality; }

  bool getHasRGBCameras() const override { return has_rgb_cameras; }

  bool getPublishRGBImages() const override { return publish_rgb_images; }

  bool getPublishDepthImages() const override { return publish_depth_images; }

  bool getPublishDepthRegisteredImages() const override { return publish_depth_registered_images; }

  std::string getSpotName() const override { return spot_name; }

  std::string address;
  std::string username;
  std::string password;

  double rgb_image_quality = kDefaultRGBImageQuality;
  bool has_rgb_cameras = kDefaultHasRGBCameras;
  bool publish_rgb_images = kDefaultPublishRGBImages;
  bool publish_depth_images = kDefaultPublishDepthImages;
  bool publish_depth_registered_images = kDefaultPublishDepthRegisteredImages;
  std::string spot_name;
};

class MockSpotApi : public SpotApi {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), createRobot, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<void, std::string>), authenticate, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<bool, std::string>), hasArm, (), (const, override));
  MOCK_METHOD(std::shared_ptr<ImageClientApi>, image_client_api, (), (const, override));
};

class SpotImagePubNodeTestFixture : public ::testing::Test {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_image_publisher_node");
    fake_parameter_interface = std::make_shared<FakeParameterInterface>();
    mock_logger_interface = std::make_shared<MockLoggerInterface>();
    mock_spot_api = std::make_unique<MockSpotApi>();
  }

  void TearDown() override { rclcpp::shutdown(); }

 protected:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<FakeParameterInterface> fake_parameter_interface;
  std::shared_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<MockSpotApi> mock_spot_api;
};

TEST_F(SpotImagePubNodeTestFixture, Construction_Success) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
  EXPECT_CALL(*mock_spot_api, hasArm).Times(1);
  EXPECT_CALL(*mock_spot_api, image_client_api).Times(1);

  // WHEN constructing a SpotImagePublisherNode
  EXPECT_NO_THROW(
      SpotImagePublisherNode(node, std::move(mock_spot_api), fake_parameter_interface, mock_logger_interface));
}

TEST_F(SpotImagePubNodeTestFixture, Construction_Create_Robot_failure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1).WillOnce(Return(tl::make_unexpected("Create Robot Failed")));
  EXPECT_CALL(*mock_spot_api, authenticate).Times(0);
  EXPECT_CALL(*mock_spot_api, hasArm).Times(0);
  EXPECT_CALL(*mock_spot_api, image_client_api).Times(0);

  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, std::move(mock_spot_api), fake_parameter_interface, mock_logger_interface),
               std::exception);
}

TEST_F(SpotImagePubNodeTestFixture, Construction_Authentication_failure) {
  {  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
    // THEN expect the following calls in sequence
    InSequence seq;
    EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
    EXPECT_CALL(*mock_spot_api, authenticate)
        .Times(1)
        .WillOnce(Return(tl::make_unexpected("Robot Authentication Failed")));
    EXPECT_CALL(*mock_spot_api, hasArm).Times(0);
    EXPECT_CALL(*mock_spot_api, image_client_api).Times(0);
  }
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, std::move(mock_spot_api), fake_parameter_interface, mock_logger_interface),
               std::exception);
}

TEST_F(SpotImagePubNodeTestFixture, Construction_hasArm_failure) {
  {  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
    // THEN expect the following calls in sequence
    InSequence seq;
    EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
    EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
    EXPECT_CALL(*mock_spot_api, hasArm).Times(1).WillOnce(Return(tl::make_unexpected("has_arm failed")));
    EXPECT_CALL(*mock_spot_api, image_client_api).Times(0);
  }
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, std::move(mock_spot_api), fake_parameter_interface, mock_logger_interface),
               std::exception);
}

}  // namespace spot_ros2::testing
