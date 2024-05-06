// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_image_client.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_node_interface.hpp>
#include <spot_driver/mock/mock_spot_api.hpp>
#include <spot_driver/mock/mock_tf_broadcaster_interface.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <memory>
#include <stdexcept>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace spot_ros2::test {
class MockMiddlewareHandle : public images::SpotImagePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, createPublishers, (const std::set<ImageSource>& image_sources, bool, bool), (override));
  MOCK_METHOD((tl::expected<void, std::string>), publishImages,
              ((const std::map<ImageSource, ImageWithCameraInfo>&),
               (const std::map<ImageSource, CompressedImageWithCameraInfo>&)),
              (override));
};

class SpotImagePubNodeTestFixture : public ::testing::Test {
 public:
  void SetUp() override {
    mock_spot_api = std::make_unique<spot_ros2::test::MockSpotApi>();
    mock_middleware_handle = std::make_unique<MockMiddlewareHandle>();

    fake_parameter_interface = std::make_unique<FakeParameterInterface>();
    mock_logger_interface = std::make_unique<spot_ros2::test::MockLoggerInterface>();
    mock_tf_broadcaster_interface = std::make_unique<MockTfBroadcasterInterface>();
    mock_timer_interface = std::make_unique<MockTimerInterface>();
    mock_node_interface = std::make_unique<MockNodeInterface>();
  }

  std::unique_ptr<MockSpotApi> mock_spot_api;
  std::unique_ptr<MockMiddlewareHandle> mock_middleware_handle;

  std::unique_ptr<FakeParameterInterface> fake_parameter_interface;
  std::unique_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<spot_ros2::test::MockTfBroadcasterInterface> mock_tf_broadcaster_interface;
  std::unique_ptr<spot_ros2::test::MockTimerInterface> mock_timer_interface;
  std::unique_ptr<MockNodeInterface> mock_node_interface;
};

TEST_F(SpotImagePubNodeTestFixture, ConstructionSuccess) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  {
    InSequence seq;
    EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
    EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
    EXPECT_CALL(*mock_spot_api, hasArm).Times(1);
    EXPECT_CALL(*mock_spot_api, image_client_interface).Times(1);
  }

  // THEN the underlying node base interface is accessed
  EXPECT_CALL(*mock_node_interface, getNodeBaseInterface).Times(1);

  // WHEN constructing a SpotImagePublisherNode
  // THEN construction succeeds
  std::unique_ptr<images::SpotImagePublisherNode> node;
  ASSERT_NO_THROW(node = std::make_unique<images::SpotImagePublisherNode>(
                      std::move(mock_spot_api), std::move(mock_middleware_handle), std::move(fake_parameter_interface),
                      std::move(mock_logger_interface), std::move(mock_tf_broadcaster_interface),
                      std::move(mock_timer_interface), std::move(mock_node_interface)));

  // WHEN we get the underlying node base interface
  // THEN no exception is thrown
  EXPECT_NO_THROW(node->get_node_base_interface());
}

TEST_F(SpotImagePubNodeTestFixture, ConstructionCreateRobotFailure) {
  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1).WillOnce(Return(tl::make_unexpected("Create Robot Failed")));
  EXPECT_CALL(*mock_spot_api, authenticate).Times(0);
  EXPECT_CALL(*mock_spot_api, hasArm).Times(0);
  EXPECT_CALL(*mock_spot_api, image_client_interface).Times(0);

  // WHEN constructing a SpotImagePublisherNode
  // THEN the constructor throws
  EXPECT_THROW(images::SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle),
                                              std::move(fake_parameter_interface), std::move(mock_logger_interface),
                                              std::move(mock_tf_broadcaster_interface), std::move(mock_timer_interface),
                                              std::move(mock_node_interface)),
               std::runtime_error);
}

TEST_F(SpotImagePubNodeTestFixture, ConstructionAuthenticationFailure) {
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
  // THEN the constructor throws
  EXPECT_THROW(images::SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle),
                                              std::move(fake_parameter_interface), std::move(mock_logger_interface),
                                              std::move(mock_tf_broadcaster_interface), std::move(mock_timer_interface),
                                              std::move(mock_node_interface)),
               std::runtime_error);
}

TEST_F(SpotImagePubNodeTestFixture, ConstructionHasArmFailure) {
  {  // GIVEN a rclcpp::Node, MiddlewareInterface, and a SpotApi
    // THEN expect the following calls in sequence
    InSequence seq;
    EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
    EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
    EXPECT_CALL(*mock_spot_api, hasArm).Times(1).WillOnce(Return(tl::make_unexpected("has_arm failed")));
    EXPECT_CALL(*mock_spot_api, image_client_interface).Times(0);
  }
  // WHEN constructing a SpotImagePublisherNode
  // THEN the constructor throws
  EXPECT_THROW(images::SpotImagePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle),
                                              std::move(fake_parameter_interface), std::move(mock_logger_interface),
                                              std::move(mock_tf_broadcaster_interface), std::move(mock_timer_interface),
                                              std::move(mock_node_interface)),
               std::runtime_error);
}

}  // namespace spot_ros2::test
