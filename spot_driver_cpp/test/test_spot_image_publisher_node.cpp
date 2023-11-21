// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/api/spot_api.hpp>
#include <spot_driver_cpp/interfaces/middleware_interface_base.hpp>
#include <spot_driver_cpp/spot_image_publisher_node.hpp>
#include <spot_driver_cpp/test/mock_middleware_interfaces.hpp>

#include <exception>
#include <memory>
#include <optional>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace spot_ros2::testing {

class MockImageClientApi : public ImageClientApi {
 public:
  MockImageClientApi() = default;
  MOCK_METHOD((tl::expected<GetImagesResult, std::string>), getImages, (::bosdyn::api::GetImageRequest request),
              (override));
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
    node = std::make_shared<rclcpp::Node>("test_image_publisher_node", rclcpp::NodeOptions());
    mock_middleware_interface = std::make_shared<MockMiddlewareInterface>();
    mock_spot_api = std::make_unique<MockSpotApi>();
  }

 protected:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<MockMiddlewareInterface> mock_middleware_interface;
  std::unique_ptr<MockSpotApi> mock_spot_api;
};

TEST_F(SpotImagePubNodeTestFixture, Construction_Success) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_middleware_interface, logger_interface()).Times(1);
  EXPECT_CALL(*mock_middleware_interface, parameter_interface()).Times(1);
  EXPECT_CALL(*mock_spot_api, createRobot(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, hasArm()).Times(1);
  EXPECT_CALL(*mock_spot_api, image_client_api()).Times(1);
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_NO_THROW(SpotImagePublisherNode(node, mock_middleware_interface, std::move(mock_spot_api)));
}

TEST_F(SpotImagePubNodeTestFixture, Construction_Create_Robot_failure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_middleware_interface, logger_interface()).Times(1);
  EXPECT_CALL(*mock_middleware_interface, parameter_interface()).Times(1);
  EXPECT_CALL(*mock_spot_api, createRobot(_, _)).Times(1).WillOnce(Return(tl::make_unexpected("Create Robot Failed")));
  EXPECT_CALL(*mock_spot_api, authenticate(_, _)).Times(0);
  EXPECT_CALL(*mock_spot_api, hasArm()).Times(0);
  EXPECT_CALL(*mock_spot_api, image_client_api()).Times(0);
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, mock_middleware_interface, std::move(mock_spot_api)), std::exception);
}

TEST_F(SpotImagePubNodeTestFixture, Construction_Authentication_failure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_middleware_interface, logger_interface()).Times(1);
  EXPECT_CALL(*mock_middleware_interface, parameter_interface()).Times(1);
  EXPECT_CALL(*mock_spot_api, createRobot(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate(_, _))
      .Times(1)
      .WillOnce(Return(tl::make_unexpected("Robot Authentication Failed")));
  EXPECT_CALL(*mock_spot_api, hasArm()).Times(0);
  EXPECT_CALL(*mock_spot_api, image_client_api()).Times(0);
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, mock_middleware_interface, std::move(mock_spot_api)), std::exception);
}

TEST_F(SpotImagePubNodeTestFixture, Construction_GetImageClient_failure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_middleware_interface, logger_interface()).Times(1);
  EXPECT_CALL(*mock_middleware_interface, parameter_interface()).Times(1);
  EXPECT_CALL(*mock_spot_api, createRobot(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, hasArm()).Times(0);
  EXPECT_CALL(*mock_spot_api, image_client_api()).Times(0);
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, mock_middleware_interface, std::move(mock_spot_api)), std::exception);
}

TEST_F(SpotImagePubNodeTestFixture, Construction_hasArm_failure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_middleware_interface, logger_interface()).Times(1);
  EXPECT_CALL(*mock_middleware_interface, parameter_interface()).Times(1);
  EXPECT_CALL(*mock_spot_api, createRobot(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate(_, _)).Times(1);
  EXPECT_CALL(*mock_spot_api, hasArm()).Times(1).WillOnce(Return(tl::make_unexpected("has_arm failed")));
  EXPECT_CALL(*mock_spot_api, image_client_api()).Times(0);
  // WHEN constructing a SpotImagePublisherNode
  EXPECT_THROW(SpotImagePublisherNode(node, mock_middleware_interface, std::move(mock_spot_api)), std::exception);
}

}  // namespace spot_ros2::testing
