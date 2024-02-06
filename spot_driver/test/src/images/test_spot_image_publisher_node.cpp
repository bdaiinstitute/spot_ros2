// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <spot_driver/mock/mock_image_client.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_spot_api.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <exception>
#include <memory>
#include <optional>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace spot_ros2::images::test {

constexpr auto kExampleAddress{"192.168.0.10"};
constexpr auto kExampleUsername{"spot_user"};
constexpr auto kExamplePassword{"hunter2"};

constexpr auto kSomeErrorMessage = "some error message";

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

class MockMiddlewareHandle : public SpotImagePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, createPublishers, (const std::set<ImageSource>& image_sources), (override));
  MOCK_METHOD((tl::expected<void, std::string>), publishImages, ((const std::map<ImageSource, ImageWithCameraInfo>&)),
              (override));
  MOCK_METHOD(std::shared_ptr<rclcpp::Node>, node, (), (override));

  MOCK_METHOD(ParameterInterfaceBase*, parameter_interface, (), (override));
  MOCK_METHOD(LoggerInterfaceBase*, logger_interface, (), (override));
  TfInterfaceBase* tf_interface() override { return tf_interface_.get(); }
  TimerInterfaceBase* timer_interface() override { return timer_interface_.get(); }

  std::unique_ptr<FakeParameterInterface> parameter_interface_ = std::make_unique<FakeParameterInterface>();
  std::unique_ptr<spot_ros2::test::MockLoggerInterface> logger_interface_ =
      std::make_unique<spot_ros2::test::MockLoggerInterface>();
  std::unique_ptr<spot_ros2::test::MockTfInterface> tf_interface_ =
      std::make_unique<spot_ros2::test::MockTfInterface>();
  std::unique_ptr<spot_ros2::test::MockTimerInterface> timer_interface_ =
      std::make_unique<spot_ros2::test::MockTimerInterface>();
};

class SpotImagePubNodeTestFixture : public ::testing::Test {
 public:
  void SetUp() override {
    fake_parameter_interface = std::make_shared<FakeParameterInterface>();
    mock_logger_interface = std::make_shared<spot_ros2::test::MockLoggerInterface>();
    mock_spot_api = std::make_unique<spot_ros2::test::MockSpotApi>();
    mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();

    ON_CALL(*mock_middleware_handle, parameter_interface()).WillByDefault(Return(fake_parameter_interface.get()));
    ON_CALL(*mock_middleware_handle, logger_interface()).WillByDefault(Return(mock_logger_interface.get()));
  }

 protected:
  std::shared_ptr<FakeParameterInterface> fake_parameter_interface;
  std::shared_ptr<spot_ros2::test::MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<spot_ros2::test::MockSpotApi> mock_spot_api;
  std::unique_ptr<spot_ros2::images::test::MockMiddlewareHandle> mock_middleware_handle;
};

TEST_F(SpotImagePubNodeTestFixture, Construction_Success) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
  EXPECT_CALL(*mock_spot_api, hasArm).Times(1);
  EXPECT_CALL(*mock_spot_api, image_client_interface).Times(1);

  // WHEN constructing a SpotImagePublisherNode
  EXPECT_NO_THROW(SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)));
}

TEST_F(SpotImagePubNodeTestFixture, Construction_Create_Robot_failure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1).WillOnce(Return(tl::make_unexpected("Create Robot Failed")));
  EXPECT_CALL(*mock_spot_api, authenticate).Times(0);
  EXPECT_CALL(*mock_spot_api, hasArm).Times(0);
  EXPECT_CALL(*mock_spot_api, image_client_interface).Times(0);

  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)), std::exception);
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
    EXPECT_CALL(*mock_spot_api, image_client_interface).Times(0);
  }
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)), std::exception);
}

TEST_F(SpotImagePubNodeTestFixture, Construction_hasArm_failure) {
  {  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
    // THEN expect the following calls in sequence
    InSequence seq;
    EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
    EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
    EXPECT_CALL(*mock_spot_api, hasArm).Times(1).WillOnce(Return(tl::make_unexpected("has_arm failed")));
    EXPECT_CALL(*mock_spot_api, image_client_interface).Times(0);
  }
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)), std::exception);
}

}  // namespace spot_ros2::images::test
