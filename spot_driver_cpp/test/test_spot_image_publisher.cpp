// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/spot_image_publisher.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
#include <spot_driver_cpp/test/mock_middleware_interfaces.hpp>
#include <spot_driver_cpp/types.hpp>

#include <memory>
#include <optional>
#include <tl_expected/expected.hpp>

using ::testing::_;
using ::testing::AllOf;
using ::testing::AtLeast;
using ::testing::HasSubstr;
using ::testing::InSequence;
using ::testing::Property;
using ::testing::Return;

namespace spot_ros2::testing {

class MockImageClientApi : public ImageClientApi {
 public:
  MOCK_METHOD((tl::expected<GetImagesResult, std::string>), getImages, (::bosdyn::api::GetImageRequest), (override));
};

class TestInitSpotImagePublisherParametersUnset : public ::testing::Test {
 public:
  void SetUp() override {
    parameter_interface_ptr = parameter_interface.get();
    timer_interface_ptr = timer_interface.get();

    publisher_interface_ptr = publisher_interface.get();
    image_client_api_ptr = image_client_api.get();
    tf_interface_ptr = tf_interface.get();
    logger_interface_ptr = logger_interface.get();

    ON_CALL(*middleware_interface, parameter_interface()).WillByDefault(Return(parameter_interface_ptr));
    ON_CALL(*middleware_interface, timer_interface()).WillByDefault(Return(timer_interface_ptr));
    ON_CALL(*middleware_interface, publisher_interface()).WillByDefault(Return(publisher_interface_ptr));
    ON_CALL(*middleware_interface, tf_interface()).WillByDefault(Return(tf_interface_ptr));
    ON_CALL(*middleware_interface, logger_interface()).WillByDefault(Return(logger_interface_ptr));
  }

  void create_image_publisher(bool has_arm) {
    image_publisher = std::make_unique<SpotImagePublisher>(std::move(image_client_api), middleware_interface, has_arm);
  }

  std::unique_ptr<FakeParameterInterface> parameter_interface = std::make_unique<FakeParameterInterface>();
  FakeParameterInterface* parameter_interface_ptr;

  std::unique_ptr<MockTimerInterface> timer_interface = std::make_unique<MockTimerInterface>();
  MockTimerInterface* timer_interface_ptr;

  std::unique_ptr<MockPublisherInterface> publisher_interface = std::make_unique<MockPublisherInterface>();
  MockPublisherInterface* publisher_interface_ptr;

  std::shared_ptr<MockImageClientApi> image_client_api = std::make_shared<MockImageClientApi>();
  MockImageClientApi* image_client_api_ptr;

  std::unique_ptr<MockTfInterface> tf_interface = std::make_unique<MockTfInterface>();
  MockTfInterface* tf_interface_ptr;

  std::unique_ptr<MockLoggerInterface> logger_interface = std::make_unique<MockLoggerInterface>();
  MockLoggerInterface* logger_interface_ptr;

  std::unique_ptr<SpotImagePublisher> image_publisher;
  std::shared_ptr<MockMiddlewareInterface> middleware_interface;
};

class TestInitSpotImagePublisher : public TestInitSpotImagePublisherParametersUnset {
 public:
  void SetUp() override {
    TestInitSpotImagePublisherParametersUnset::SetUp();

    parameter_interface_ptr->address = kExampleAddress;
    parameter_interface_ptr->username = kExampleUsername;
    parameter_interface_ptr->password = kExamplePassword;
  }
};

class TestRunSpotImagePublisher : public TestInitSpotImagePublisher {
 public:
  void SetUp() override {
    parameter_interface_ptr = parameter_interface.get();
    timer_interface_ptr = timer_interface.get();

    publisher_interface_ptr = publisher_interface.get();
    image_client_api_ptr = image_client_api.get();
    tf_interface_ptr = tf_interface.get();
    logger_interface_ptr = logger_interface.get();

    parameter_interface_ptr->address = kExampleAddress;
    parameter_interface_ptr->username = kExampleUsername;
    parameter_interface_ptr->password = kExamplePassword;

    ON_CALL(*middleware_interface, parameter_interface()).WillByDefault(Return(parameter_interface_ptr));
    ON_CALL(*middleware_interface, timer_interface()).WillByDefault(Return(timer_interface_ptr));
    ON_CALL(*middleware_interface, publisher_interface()).WillByDefault(Return(publisher_interface_ptr));
    ON_CALL(*middleware_interface, tf_interface()).WillByDefault(Return(tf_interface_ptr));
    ON_CALL(*middleware_interface, logger_interface()).WillByDefault(Return(logger_interface_ptr));
  }

  std::unique_ptr<FakeParameterInterface> parameter_interface = std::make_unique<FakeParameterInterface>();
  FakeParameterInterface* parameter_interface_ptr;

  std::unique_ptr<FakeTimerInterface> timer_interface = std::make_unique<FakeTimerInterface>();
  FakeTimerInterface* timer_interface_ptr;

  std::unique_ptr<MockPublisherInterface> publisher_interface = std::make_unique<MockPublisherInterface>();
  MockPublisherInterface* publisher_interface_ptr;

  std::shared_ptr<MockImageClientApi> image_client_api = std::make_shared<MockImageClientApi>();
  MockImageClientApi* image_client_api_ptr;

  std::unique_ptr<MockTfInterface> tf_interface = std::make_unique<MockTfInterface>();
  MockTfInterface* tf_interface_ptr;

  std::unique_ptr<MockLoggerInterface> logger_interface = std::make_unique<MockLoggerInterface>();
  MockLoggerInterface* logger_interface_ptr;

  std::unique_ptr<SpotImagePublisher> image_publisher;
  std::shared_ptr<MockMiddlewareInterface> middleware_interface;
};

TEST_F(TestInitSpotImagePublisher, InitSucceeds) {
  // GIVEN all required parameters are set to correct values
  // GIVEN an image publisher that is expected to publish gripper camera images
  const auto has_arm{true};
  create_image_publisher(has_arm);

  // THEN the timer interface's setTimer function is called once with the expected timer period
  EXPECT_CALL(*timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0 / 15.0}, _)).Times(1);

  // WHEN the SpotImagePublisher is initialized
  // THEN initialization succeeds
  EXPECT_TRUE(image_publisher->initialize());
}

TEST_F(TestRunSpotImagePublisher, PublishCallbackTriggersWithArm) {
  // GIVEN we request all possible image types
  parameter_interface_ptr->publish_rgb_images = true;
  parameter_interface_ptr->publish_depth_images = true;
  parameter_interface_ptr->publish_depth_registered_images = true;

  // GIVEN an image_publisher
  const auto has_arm{true};
  create_image_publisher(has_arm);

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras + 1 hand camera = 18 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*image_client_api_ptr, getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 18)));
    EXPECT_CALL(*publisher_interface_ptr, publish);
    EXPECT_CALL(*tf_interface_ptr, updateStaticTransforms);
  }

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

TEST_F(TestRunSpotImagePublisher, PublishCallbackTriggersWithNoArm) {
  // GIVEN we request all possible image types
  parameter_interface_ptr->publish_rgb_images = true;
  parameter_interface_ptr->publish_depth_images = true;
  parameter_interface_ptr->publish_depth_registered_images = true;

  // GIVEN an image publisher not expected to publish camera data
  const auto has_arm{false};
  create_image_publisher(has_arm);

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras = 15 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*image_client_api_ptr, getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 15)));
    EXPECT_CALL(*publisher_interface_ptr, publish);
    EXPECT_CALL(*tf_interface_ptr, updateStaticTransforms);
  }

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}
}  // namespace spot_ros2::testing
