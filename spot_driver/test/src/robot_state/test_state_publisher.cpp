// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_node_interface.hpp>
#include <spot_driver/mock/mock_state_client.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>
#include <spot_driver/robot_state/state_publisher.hpp>
#include <spot_driver/types.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tl_expected/expected.hpp>

#include <memory>

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

  std::shared_ptr<spot_ros2::test::MockStateClient> mock_state_client_interface =
      std::make_shared<spot_ros2::test::MockStateClient>();
  std::unique_ptr<MockStateMiddlewareHandle> mock_middleware_handle = std::make_unique<MockStateMiddlewareHandle>();
  std::unique_ptr<StatePublisher> robot_state_publisher;
};

RobotState makeRobotState(const bool has_valid_tf = true) {
  RobotState out;
  if (has_valid_tf) {
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped transform;
    tf_msg.transforms.push_back(transform);
    out.maybe_tf = tf_msg;
  }
  return out;
}

TEST_F(StatePublisherNodeTest, InitSucceeds) {
  // GIVEN a RobotStateClientInterface and a StatePublisher::MiddlewareHandle

  // THEN the timer interface's setTimer function is called once with the expected timer period
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0 / 50.0}, _)).Times(1);

  // WHEN a robot state publisher is constructed
  robot_state_publisher = std::make_unique<StatePublisher>(
      mock_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));
}

TEST_F(StatePublisherNodeTest, PublishCallbackTriggers) {
  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  {
    InSequence seq;
    // GIVEN the robot state will contain transforms
    // THEN we request the robot state from the Spot interface
    EXPECT_CALL(*mock_state_client_interface, getRobotState)
        .WillOnce(Return(tl::expected<RobotState, std::string>{makeRobotState(true)}));
    // THEN we publish the robot state to the appropriate topics
    EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(1);
    // THEN the robot transforms are published to TF
    EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(1);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher = std::make_unique<StatePublisher>(
      mock_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

TEST_F(StatePublisherNodeTest, PublishCallbackTriggersNoTfData) {
  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  {
    InSequence seq;
    // GIVEN the robot state does not contain any transforms
    // THEN we request the robot state from the Spot interface
    EXPECT_CALL(*mock_state_client_interface, getRobotState)
        .WillOnce(Return(tl::expected<RobotState, std::string>(makeRobotState(false))));
    // THEN we publish the robot state to the appropriate topics
    EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(1);
    // THEN no transforms are published to TF
    EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(0);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher = std::make_unique<StatePublisher>(
      mock_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

TEST_F(StatePublisherNodeTest, PublishCallbackTriggersFailGetRobotState) {
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
    EXPECT_CALL(*mock_state_client_interface, getRobotState)
        .Times(1)
        .WillOnce(Return(tl::make_unexpected("Failed to get robot state")));
    // THEN an error message is logged
    EXPECT_CALL(*logger_interface_ptr, logError).Times(1);
    // THEN we do not publish a robot state
    EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(0);
    // THEN we do not publish to TF
    EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(0);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher = std::make_unique<StatePublisher>(
      mock_state_client_interface, std::move(mock_middleware_handle), std::move(fake_parameter_interface),
      std::move(mock_logger_interface), std::move(mock_tf_interface), std::move(mock_timer_interface));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

}  // namespace spot_ros2::test
