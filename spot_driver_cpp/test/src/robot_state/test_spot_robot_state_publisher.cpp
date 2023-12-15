// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/robot_state/spot_robot_state_publisher.hpp>

#include <spot_driver_cpp/mock/mock_logger_interface.hpp>
#include <spot_driver_cpp/mock/mock_robot_state_client.hpp>
#include <spot_driver_cpp/mock/mock_tf_interface.hpp>
#include <spot_driver_cpp/mock/mock_timer_interface.hpp>

#include <memory>
#include <optional>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::AllOf;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Property;
using ::testing::Return;
using ::testing::Unused;

namespace spot_ros2::test {

constexpr auto kExampleAddress{"192.168.0.10"};
constexpr auto kExampleUsername{"spot_user"};
constexpr auto kExamplePassword{"hunter2"};

constexpr auto kSomeErrorMessage = "some error message";

class FakeParameterInterface : public ParameterInterfaceBase {
 public:
  std::string getAddress() const override { return kExampleAddress; }

  std::string getUsername() const override { return kExampleUsername; }

  std::string getPassword() const override { return kExamplePassword; }

  double getRGBImageQuality() const override { return rgb_image_quality; }

  bool getHasRGBCameras() const override { return has_rgb_cameras; }

  bool getPublishRGBImages() const override { return publish_rgb_images; }

  bool getPublishDepthImages() const override { return publish_depth_images; }

  bool getPublishDepthRegisteredImages() const override { return publish_depth_registered_images; }

  std::string getPreferredOdomFrame() const override { return "odom"; }

  std::string getSpotName() const override { return spot_name; }

  double rgb_image_quality = kDefaultRGBImageQuality;
  bool has_rgb_cameras = kDefaultHasRGBCameras;
  bool publish_rgb_images = kDefaultPublishRGBImages;
  bool publish_depth_images = kDefaultPublishDepthImages;
  bool publish_depth_registered_images = kDefaultPublishDepthRegisteredImages;
  std::string spot_name;
};

class MockRobotMiddlewareHandle : public SpotRobotStatePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, createPublishers, (), (override));
  MOCK_METHOD(void, publishRobotState, (const RobotState& robot_state), (override));
  MOCK_METHOD(std::shared_ptr<rclcpp::Node>, node, (), (override));

  ParameterInterfaceBase* parameter_interface() override { return parameter_interface_.get(); }
  LoggerInterfaceBase* logger_interface() override { return logger_interface_.get(); }
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

class TestSpotRobotStatePublisherFixture : public ::testing::Test {
 public:
  std::shared_ptr<spot_ros2::test::MockRobotStateClient> robot_state_client_interface =
      std::make_shared<spot_ros2::test::MockRobotStateClient>();
  std::unique_ptr<MockRobotMiddlewareHandle> middleware_handle = std::make_unique<MockRobotMiddlewareHandle>();
  std::unique_ptr<SpotRobotStatePublisher> robot_state_publisher;
};

TEST_F(TestSpotRobotStatePublisherFixture, InitSucceeds) {
  // GIVEN a RobotStateClientInterface and a SpotRobotStatePublisher::MiddlewareHandle

  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);
  // THEN the timer interface's setTimer function is called once with the expected timer period
  auto timer_interface_ptr = middleware_handle->timer_interface_.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0 / 50.0}, _)).Times(1);

  // WHEN a robot state publisher is constructed
  robot_state_publisher =
      std::make_unique<SpotRobotStatePublisher>(robot_state_client_interface, std::move(middleware_handle));
}

TEST_F(TestSpotRobotStatePublisherFixture, PublishCallbackTriggers) {
  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto timer_interface_ptr = middleware_handle->timer_interface_.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  {
    // THEN we request the robot state from the Spot interface
    // THEN we publish the robot state to the appropriate topics
    InSequence seq;
    EXPECT_CALL(*robot_state_client_interface, getRobotState);
    EXPECT_CALL(*middleware_handle, publishRobotState);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher =
      std::make_unique<SpotRobotStatePublisher>(robot_state_client_interface, std::move(middleware_handle));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

TEST_F(TestSpotRobotStatePublisherFixture, PublishCallbackTriggers_Fail_GetRobotState) {
  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto timer_interface_ptr = middleware_handle->timer_interface_.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  auto logger_interface_ptr = middleware_handle->logger_interface_.get();
  {
    // THEN we request the robot state from the Spot interface
    // THEN we publish the robot state to the appropriate topics
    InSequence seq;
    EXPECT_CALL(*robot_state_client_interface, getRobotState)
        .Times(1)
        .WillOnce(Return(tl::make_unexpected("Failed to get robot state")));
    EXPECT_CALL(*logger_interface_ptr, logError).Times(1);
    EXPECT_CALL(*middleware_handle, publishRobotState).Times(0);
  }

  // GIVEN a robot_state_publisher
  robot_state_publisher =
      std::make_unique<SpotRobotStatePublisher>(robot_state_client_interface, std::move(middleware_handle));

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

}  // namespace spot_ros2::test
