// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/robot_state/state_publisher.hpp>

#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_node_interface.hpp>
#include <spot_driver/mock/mock_robot_state_client.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>

#include <memory>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::AllOf;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Property;
using ::testing::Return;
using ::testing::Unused;

namespace spot_ros2::test {
class MockStateMiddlewareHandle : public StatePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, publishRobotState, (const RobotState& robot_state), (override));
};

class StatePublisherNodeTest : public ::testing::Test {
 public:
  void SetUp() override {
    mock_node_interface = std::make_unique<MockNodeInterface>();

    fake_parameter_interface = std::make_unique<FakeParameterInterface>();
    mock_logger_interface = std::make_unique<spot_ros2::test::MockLoggerInterface>();
    mock_tf_interface = std::make_unique<spot_ros2::test::MockTfInterface>();
    mock_timer_interface = std::make_unique<spot_ros2::test::MockTimerInterface>();
  }

  std::unique_ptr<MockNodeInterface> mock_node_interface;
  std::unique_ptr<FakeParameterInterface> fake_parameter_interface;
  std::unique_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<MockTfInterface> mock_tf_interface;
  std::unique_ptr<MockTimerInterface> mock_timer_interface;

  std::shared_ptr<spot_ros2::test::MockRobotStateClient> robot_state_client_interface =
      std::make_shared<spot_ros2::test::MockRobotStateClient>();
  std::unique_ptr<MockStateMiddlewareHandle> mock_middleware_handle = std::make_unique<MockStateMiddlewareHandle>();
  std::unique_ptr<StatePublisher> robot_state_publisher;
};

TEST_F(StatePublisherNodeTest, InitSucceeds) {
  // GIVEN a RobotStateClientInterface and a StatePublisher::MiddlewareHandle

  // THEN the timer interface's setTimer function is called once with the expected timer period
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0 / 50.0}, _)).Times(1);

  // WHEN a robot state publisher is constructed
  robot_state_publisher = std::make_unique<StatePublisher>(
      robot_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));
}

TEST_F(StatePublisherNodeTest, PublishCallbackTriggers) {
  // THEN expect createPublishers to be invoked

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  {
    InSequence seq;
    // THEN we request the robot state from the Spot interface
    EXPECT_CALL(*robot_state_client_interface, getRobotState);
    // THEN we publish the robot state to the appropriate topics
    EXPECT_CALL(*mock_middleware_handle, publishRobotState);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher = std::make_unique<StatePublisher>(
      robot_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

TEST_F(StatePublisherNodeTest, PublishCallbackTriggersFailGetRobotState) {
  // THEN expect createPublishers to be invoked

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  auto* logger_interface_ptr = mock_logger_interface.get();
  {
    InSequence seq;
    // GIVEN the request to retrieve the robot state will fail
    // THEN we request the robot state from the Spot interface
    EXPECT_CALL(*robot_state_client_interface, getRobotState)
        .Times(1)
        .WillOnce(Return(tl::make_unexpected("Failed to get robot state")));
    // THEN an error message is logged
    EXPECT_CALL(*logger_interface_ptr, logError).Times(1);
    // THEN we publish the robot state to the appropriate topics
    EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(0);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher = std::make_unique<StatePublisher>(
      robot_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

}  // namespace spot_ros2::test
