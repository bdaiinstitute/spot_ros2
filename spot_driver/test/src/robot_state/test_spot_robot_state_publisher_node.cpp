// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/robot_state/spot_robot_state_publisher_node.hpp>

#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_robot_state_client.hpp>
#include <spot_driver/mock/mock_spot_api.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <exception>
#include <memory>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace spot_ros2::test {

class MockRobotMiddlewareHandle : public SpotRobotStatePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, createPublishers, (), (override));
  MOCK_METHOD(void, publishRobotState, (const RobotState& robot_state), (override));

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

class SpotRobotStatePubNodeTestFixture : public ::testing::Test {
 public:
  void SetUp() override {
    fake_parameter_interface = std::make_shared<FakeParameterInterface>();
    mock_logger_interface = std::make_shared<spot_ros2::test::MockLoggerInterface>();
    mock_spot_api = std::make_unique<spot_ros2::test::MockSpotApi>();
    mock_middleware_handle = std::make_unique<MockRobotMiddlewareHandle>();

    ON_CALL(*mock_middleware_handle, parameter_interface()).WillByDefault(Return(fake_parameter_interface.get()));
    ON_CALL(*mock_middleware_handle, logger_interface()).WillByDefault(Return(mock_logger_interface.get()));
  }

  std::shared_ptr<FakeParameterInterface> fake_parameter_interface;
  std::shared_ptr<spot_ros2::test::MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<spot_ros2::test::MockSpotApi> mock_spot_api;
  std::unique_ptr<spot_ros2::test::MockRobotMiddlewareHandle> mock_middleware_handle;
};

TEST_F(SpotRobotStatePubNodeTestFixture, ConstructionSuccessful) {
  // GIVEN a MiddlewareInterface and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate).Times(1);
  EXPECT_CALL(*mock_spot_api, robot_state_client_interface).Times(1);
  EXPECT_CALL(*mock_logger_interface, logError).Times(0);

  // WHEN constructing a SpotRobotStatePublisherNode
  EXPECT_NO_THROW(SpotRobotStatePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)));
}

TEST_F(SpotRobotStatePubNodeTestFixture, ConstructionFailedCreateRobotFailure) {
  // GIVEN MiddlewareInterface and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1).WillOnce(Return(tl::make_unexpected("Create Robot Failed")));
  EXPECT_CALL(*mock_logger_interface, logError).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate).Times(0);
  EXPECT_CALL(*mock_spot_api, robot_state_client_interface).Times(0);

  // WHEN constructing a SpotRobotStatePublisherNode
  EXPECT_THROW(SpotRobotStatePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)),
               std::exception);
}

TEST_F(SpotRobotStatePubNodeTestFixture, ConstructionFailedAuthenticateFailure) {
  // GIVEN a MiddlewareInterface and a SpotApi
  // THEN expect the following calls in sequence
  InSequence seq;
  EXPECT_CALL(*mock_spot_api, createRobot).Times(1);
  EXPECT_CALL(*mock_spot_api, authenticate).Times(1).WillOnce(Return(tl::make_unexpected("Authenication Failed")));
  EXPECT_CALL(*mock_logger_interface, logError).Times(1);
  EXPECT_CALL(*mock_spot_api, robot_state_client_interface).Times(0);

  // WHEN constructing a SpotRobotStatePublisherNode
  EXPECT_THROW(SpotRobotStatePublisherNode(std::move(mock_spot_api), std::move(mock_middleware_handle)),
               std::exception);
}

}  // namespace spot_ros2::test
