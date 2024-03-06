// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/images/spot_image_publisher.hpp>
#include <spot_driver/types.hpp>

#include <spot_driver/mock/mock_image_client.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>

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

namespace spot_ros2::images::test {

constexpr auto kExampleAddress{"192.168.0.10"};
constexpr auto kExampleUsername{"spot_user"};
constexpr auto kExamplePassword{"hunter2"};

constexpr auto kSomeErrorMessage = "some error message";

class FakeParameterInterface : public ParameterInterfaceBase {
 public:
  std::string getHostname() const override { return kExampleAddress; }

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

class MockMiddlewareHandle : public SpotImagePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, createPublishers, (const std::set<ImageSource>& image_sources), (override));
  MOCK_METHOD((tl::expected<void, std::string>), publishImages, ((const std::map<ImageSource, ImageWithCameraInfo>&)),
              (override));
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

class TestInitSpotImagePublisher : public ::testing::Test {
 public:
  void create_image_publisher(bool has_arm) {
    image_publisher =
        std::make_unique<SpotImagePublisher>(image_client_interface, std::move(middleware_handle), has_arm);
  }

  std::shared_ptr<spot_ros2::test::MockImageClient> image_client_interface =
      std::make_shared<spot_ros2::test::MockImageClient>();
  std::unique_ptr<MockMiddlewareHandle> middleware_handle = std::make_unique<MockMiddlewareHandle>();
  std::unique_ptr<SpotImagePublisher> image_publisher;
};

class TestRunSpotImagePublisher : public TestInitSpotImagePublisher {};

TEST_F(TestInitSpotImagePublisher, InitSucceeds) {
  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);
  // THEN the timer interface's setTimer function is called once with the expected timer period
  EXPECT_CALL(*middleware_handle->timer_interface_, setTimer(std::chrono::duration<double>{1.0 / 15.0}, _)).Times(1);

  // GIVEN all required parameters are set to correct values
  // GIVEN an image publisher that is expected to publish gripper camera images
  const auto has_arm{true};
  create_image_publisher(has_arm);

  // WHEN the SpotImagePublisher is initialized
  // THEN initialization succeeds
  image_publisher->initialize();
}

TEST_F(TestRunSpotImagePublisher, PublishCallbackTriggersWithArm) {
  // GIVEN we request all possible image types
  middleware_handle->parameter_interface_->publish_rgb_images = true;
  middleware_handle->parameter_interface_->publish_depth_images = true;
  middleware_handle->parameter_interface_->publish_depth_registered_images = true;

  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto timer_interface_ptr = middleware_handle->timer_interface_.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras + 1 hand camera = 18 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*image_client_interface, getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 18)));
    EXPECT_CALL(*middleware_handle, publishImages);
    EXPECT_CALL(*middleware_handle->tf_interface_, updateStaticTransforms);
  }

  // GIVEN an image_publisher
  const auto has_arm{true};
  create_image_publisher(has_arm);

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

TEST_F(TestRunSpotImagePublisher, PublishCallbackTriggersWithNoArm) {
  // GIVEN we request all possible image types
  middleware_handle->parameter_interface_->publish_rgb_images = true;
  middleware_handle->parameter_interface_->publish_depth_images = true;
  middleware_handle->parameter_interface_->publish_depth_registered_images = true;

  // THEN expect createPublishers to be invoked
  EXPECT_CALL(*middleware_handle, createPublishers).Times(1);

  // THEN the timer interface's setTimer function is called once and the timer_callback is set
  auto timer_interface_ptr = middleware_handle->timer_interface_.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras = 15 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*image_client_interface, getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 15)));
    EXPECT_CALL(*middleware_handle, publishImages);
    EXPECT_CALL(*middleware_handle->tf_interface_, updateStaticTransforms);
  }

  // GIVEN an image publisher not expected to publish camera data
  const auto has_arm{false};
  create_image_publisher(has_arm);

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}
}  // namespace spot_ros2::images::test
