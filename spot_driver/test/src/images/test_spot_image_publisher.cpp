// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/images/spot_image_publisher.hpp>
#include <spot_driver/types.hpp>

#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_image_client.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_node_interface.hpp>
#include <spot_driver/mock/mock_spot_api.hpp>
#include <spot_driver/mock/mock_tf_broadcaster_interface.hpp>
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
class MockMiddlewareHandle : public images::SpotImagePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, createPublishers, (const std::set<ImageSource>& image_sources, bool, bool), (override));
  MOCK_METHOD((tl::expected<void, std::string>), publishImages,
              ((const std::map<ImageSource, ImageWithCameraInfo>&),
               (const std::map<ImageSource, CompressedImageWithCameraInfo>&)),
              (override));
};

class TestInitSpotImagePublisher : public ::testing::Test {
 public:
  void SetUp() override {
    image_client_interface = std::make_shared<spot_ros2::test::MockImageClient>();
    middleware_handle = std::make_unique<MockMiddlewareHandle>();
    fake_parameter_interface = std::make_unique<FakeParameterInterface>();
    mock_logger_interface = std::make_unique<spot_ros2::test::MockLoggerInterface>();
    mock_tf_broadcaster_interface = std::make_unique<MockTfBroadcasterInterface>();
    mock_timer_interface = std::make_unique<MockTimerInterface>();

    middleware_handle_ptr = middleware_handle.get();
    fake_parameter_interface_ptr = fake_parameter_interface.get();
    mock_logger_interface_ptr = mock_logger_interface.get();
    mock_tf_broadcaster_interface_ptr = mock_tf_broadcaster_interface.get();
    mock_timer_interface_ptr = mock_timer_interface.get();
  }

  void createImagePublisher(bool has_arm) {
    image_publisher = std::make_unique<images::SpotImagePublisher>(
        image_client_interface, std::move(middleware_handle), std::move(fake_parameter_interface),
        std::move(mock_logger_interface), std::move(mock_tf_broadcaster_interface), std::move(mock_timer_interface),
        has_arm);
  }

  std::unique_ptr<images::SpotImagePublisher> image_publisher;

  std::shared_ptr<spot_ros2::test::MockImageClient> image_client_interface;
  std::unique_ptr<MockMiddlewareHandle> middleware_handle;
  std::unique_ptr<FakeParameterInterface> fake_parameter_interface;
  std::unique_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<spot_ros2::test::MockTfBroadcasterInterface> mock_tf_broadcaster_interface;
  std::unique_ptr<spot_ros2::test::MockTimerInterface> mock_timer_interface;

  MockMiddlewareHandle* middleware_handle_ptr = nullptr;
  FakeParameterInterface* fake_parameter_interface_ptr = nullptr;
  MockLoggerInterface* mock_logger_interface_ptr = nullptr;
  MockTfBroadcasterInterface* mock_tf_broadcaster_interface_ptr = nullptr;
  MockTimerInterface* mock_timer_interface_ptr = nullptr;
};

class TestRunSpotImagePublisher : public TestInitSpotImagePublisher {};

TEST_F(TestInitSpotImagePublisher, InitSucceeds) {
  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);
  // THEN the timer interface's setTimer function is called once with the expected timer period
  EXPECT_CALL(*mock_timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0 / 15.0}, _)).Times(1);

  // GIVEN all required parameters are set to correct values
  // GIVEN an image publisher that is expected to publish gripper camera images
  constexpr auto kHasArm{true};
  createImagePublisher(kHasArm);

  // WHEN the SpotImagePublisher is initialized
  // THEN initialization succeeds
  EXPECT_THAT(image_publisher->initialize(), testing::IsTrue());
}

TEST_F(TestRunSpotImagePublisher, PublishCallbackTriggersWithArm) {
  // GIVEN we request all possible image types
  fake_parameter_interface_ptr->publish_rgb_images = true;
  fake_parameter_interface_ptr->publish_depth_images = true;
  fake_parameter_interface_ptr->publish_depth_registered_images = true;

  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  EXPECT_CALL(*mock_timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    mock_timer_interface_ptr->onSetTimer(cb);
  });

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras + 1 hand camera = 18 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*image_client_interface,
                getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 18), true, false));
    EXPECT_CALL(*middleware_handle, publishImages);
    EXPECT_CALL(*mock_tf_broadcaster_interface_ptr, updateStaticTransforms);
  }

  // GIVEN an image_publisher
  constexpr auto kHasArm{true};
  createImagePublisher(kHasArm);

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  // WHEN the timer callback is triggered
  mock_timer_interface_ptr->trigger();
}

TEST_F(TestRunSpotImagePublisher, PublishCallbackTriggersWithNoArm) {
  // GIVEN we request all possible image types
  fake_parameter_interface_ptr->publish_rgb_images = true;
  fake_parameter_interface_ptr->publish_depth_images = true;
  fake_parameter_interface_ptr->publish_depth_registered_images = true;

  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  EXPECT_CALL(*mock_timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    mock_timer_interface_ptr->onSetTimer(cb);
  });

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras = 15 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*image_client_interface,
                getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 15), true, false));
    EXPECT_CALL(*middleware_handle, publishImages);
    EXPECT_CALL(*mock_tf_broadcaster_interface_ptr, updateStaticTransforms);
  }

  // GIVEN an image publisher not expected to publish camera data
  constexpr auto kHasArm{false};
  createImagePublisher(kHasArm);

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  // WHEN the timer callback is triggered
  mock_timer_interface_ptr->trigger();
}
}  // namespace spot_ros2::test
