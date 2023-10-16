// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>
#include <spot_driver_cpp/interfaces/publisher_interface_base.hpp>
#include <spot_driver_cpp/interfaces/spot_interface_base.hpp>
#include <spot_driver_cpp/interfaces/tf_interface_base.hpp>
#include <spot_driver_cpp/interfaces/timer_interface_base.hpp>
#include <spot_driver_cpp/spot_image_publisher.hpp>
#include <spot_driver_cpp/spot_image_sources.hpp>
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

class FakeTimerInterface : public TimerInterfaceBase {
 public:
  void setTimer([[maybe_unused]] const std::chrono::duration<double>& period,
                const std::function<void()>& callback) override {
    callback_ = callback;
  }

  void clearTimer() override {}

  void trigger() { callback_(); }

 private:
  std::function<void()> callback_;
};

class MockPublisherInterface : public PublisherInterfaceBase {
 public:
  MOCK_METHOD(void, createPublishers, (const std::set<ImageSource>& image_sources), (override));
  MOCK_METHOD((tl::expected<void, std::string>), publish, ((const std::map<ImageSource, ImageWithCameraInfo>&)),
              (override));
};

class MockSpotInterface : public SpotInterfaceBase {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), createRobot, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<void, std::string>), authenticate,
              (const std::string& username, const std::string& password), (override));
  MOCK_METHOD((tl::expected<bool, std::string>), hasArm, (), (const, override));
  MOCK_METHOD((tl::expected<GetImagesResult, std::string>), getImages, (::bosdyn::api::GetImageRequest request),
              (override));
  MOCK_METHOD((tl::expected<builtin_interfaces::msg::Time, std::string>), convertRobotTimeToLocalTime,
              (const google::protobuf::Timestamp& robot_timestamp), (override));
};

class MockTimerInterface : public TimerInterfaceBase {
 public:
  MOCK_METHOD(void, setTimer, (const std::chrono::duration<double>& period, const std::function<void()>& callback),
              (override));
  MOCK_METHOD(void, clearTimer, (), (override));
};

class MockTfInterface : public TfInterfaceBase {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), updateStaticTransforms,
              (const std::vector<geometry_msgs::msg::TransformStamped>& transforms), (override));
};

class MockLoggerInterface : public LoggerInterfaceBase {
 public:
  MOCK_METHOD(void, logDebug, (const std::string& message), (const, override));
  MOCK_METHOD(void, logInfo, (const std::string& message), (const, override));
  MOCK_METHOD(void, logWarn, (const std::string& message), (const, override));
  MOCK_METHOD(void, logError, (const std::string& message), (const, override));
  MOCK_METHOD(void, logFatal, (const std::string& message), (const, override));
};

class TestInitSpotImagePublisherParametersUnset : public ::testing::Test {
 public:
  void SetUp() override {
    parameter_interface_ptr = parameter_interface.get();
    timer_interface_ptr = timer_interface.get();

    publisher_interface_ptr = publisher_interface.get();
    spot_interface_ptr = spot_interface.get();
    tf_interface_ptr = tf_interface.get();
    logger_interface_ptr = logger_interface.get();

    image_publisher = std::make_unique<SpotImagePublisher>(
        std::move(timer_interface), std::move(spot_interface), std::move(publisher_interface),
        std::move(parameter_interface), std::move(tf_interface), std::move(logger_interface));
  }

  std::unique_ptr<FakeParameterInterface> parameter_interface = std::make_unique<FakeParameterInterface>();
  FakeParameterInterface* parameter_interface_ptr;

  std::unique_ptr<MockTimerInterface> timer_interface = std::make_unique<MockTimerInterface>();
  MockTimerInterface* timer_interface_ptr;

  std::unique_ptr<MockPublisherInterface> publisher_interface = std::make_unique<MockPublisherInterface>();
  MockPublisherInterface* publisher_interface_ptr;

  std::unique_ptr<MockSpotInterface> spot_interface = std::make_unique<MockSpotInterface>();
  MockSpotInterface* spot_interface_ptr;

  std::unique_ptr<MockTfInterface> tf_interface = std::make_unique<MockTfInterface>();
  MockTfInterface* tf_interface_ptr;

  std::unique_ptr<MockLoggerInterface> logger_interface = std::make_unique<MockLoggerInterface>();
  MockLoggerInterface* logger_interface_ptr;

  std::unique_ptr<SpotImagePublisher> image_publisher;
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
    spot_interface_ptr = spot_interface.get();
    tf_interface_ptr = tf_interface.get();
    logger_interface_ptr = logger_interface.get();

    image_publisher = std::make_unique<SpotImagePublisher>(
        std::move(timer_interface), std::move(spot_interface), std::move(publisher_interface),
        std::move(parameter_interface), std::move(tf_interface), std::move(logger_interface));

    parameter_interface_ptr->address = kExampleAddress;
    parameter_interface_ptr->username = kExampleUsername;
    parameter_interface_ptr->password = kExamplePassword;

    ON_CALL(*spot_interface_ptr, createRobot).WillByDefault(Return(tl::expected<void, std::string>{}));
    ON_CALL(*spot_interface_ptr, authenticate).WillByDefault(Return(tl::expected<void, std::string>{}));
  }

  std::unique_ptr<FakeParameterInterface> parameter_interface = std::make_unique<FakeParameterInterface>();
  FakeParameterInterface* parameter_interface_ptr;

  std::unique_ptr<FakeTimerInterface> timer_interface = std::make_unique<FakeTimerInterface>();
  FakeTimerInterface* timer_interface_ptr;

  std::unique_ptr<MockPublisherInterface> publisher_interface = std::make_unique<MockPublisherInterface>();
  MockPublisherInterface* publisher_interface_ptr;

  std::unique_ptr<MockSpotInterface> spot_interface = std::make_unique<MockSpotInterface>();
  MockSpotInterface* spot_interface_ptr;

  std::unique_ptr<MockTfInterface> tf_interface = std::make_unique<MockTfInterface>();
  MockTfInterface* tf_interface_ptr;

  std::unique_ptr<MockLoggerInterface> logger_interface = std::make_unique<MockLoggerInterface>();
  MockLoggerInterface* logger_interface_ptr;

  std::unique_ptr<SpotImagePublisher> image_publisher;
};

TEST_F(TestInitSpotImagePublisher, InitFailsIfRobotNotCreated) {
  // GIVEN all required parameters are set to correct values

  // GIVEN the spot interface's createRobot function will return false to indicate that it did not succeed
  // THEN the createRobot function is called exactly once with the same address as what was provided through the
  // parameter interface
  EXPECT_CALL(*spot_interface_ptr, createRobot(kExampleAddress, _))
      .WillOnce(Return(tl::make_unexpected(kSomeErrorMessage)));
  // THEN the expected error message is logged
  EXPECT_CALL(*logger_interface_ptr,
              logError(AllOf(HasSubstr("Failed to create interface to robot:"), HasSubstr(kSomeErrorMessage))));

  // WHEN the SpotImagePublisher is initialized
  // THEN initialization fails
  ASSERT_FALSE(image_publisher->initialize());
}

TEST_F(TestInitSpotImagePublisher, InitFailsIfRobotNotAuthenticated) {
  // GIVEN all required parameters are set to correct values
  // GIVEN the createRobot function will return true to indicate that it succeeded
  ON_CALL(*spot_interface_ptr, createRobot).WillByDefault(Return(tl::expected<void, std::string>{}));

  // GIVEN the spot interface's authenticate function will return false to indicate that it failed
  // THEN the authenticate function is called exactly once with the same username and password as what was provided
  // through the parameter interface
  EXPECT_CALL(*spot_interface_ptr, authenticate(kExampleUsername, kExamplePassword))
      .WillOnce(Return(tl::make_unexpected(kSomeErrorMessage)));
  // THEN the expected error message is logged
  EXPECT_CALL(*logger_interface_ptr,
              logError(AllOf(HasSubstr("Failed to authenticate with robot:"), HasSubstr(kSomeErrorMessage))));

  // WHEN the SpotImagePulisher is initialized
  // THEN initialization fails
  ASSERT_FALSE(image_publisher->initialize());
}

TEST_F(TestInitSpotImagePublisher, InitFailsIfHasArmFails) {
  // GIVEN all required parameters are set to correct values
  // GIVEN the createRobot function will return true to indicate that it succeeded
  // GIVEN authentication will succeed
  ON_CALL(*spot_interface_ptr, createRobot).WillByDefault(Return(tl::expected<void, std::string>{}));
  ON_CALL(*spot_interface_ptr, authenticate).WillByDefault(Return(tl::expected<void, std::string>{}));

  // GIVEN the check to determine if Spot has an arm will fail
  // THEN hasArm() will be called exactly once
  EXPECT_CALL(*spot_interface_ptr, hasArm).WillOnce(Return(tl::make_unexpected(kSomeErrorMessage)));
  // THEN the expected error message is logged
  EXPECT_CALL(*logger_interface_ptr, logError(AllOf(HasSubstr("Failed to determine if Spot is equipped with an arm:"),
                                                    HasSubstr(kSomeErrorMessage))));

  // WHEN the SpotImagePulisher is initialized
  // THEN initialization fails
  ASSERT_FALSE(image_publisher->initialize());
}

TEST_F(TestInitSpotImagePublisher, InitSucceeds) {
  // GIVEN all required parameters are set to correct values
  // GIVEN the createRobot function will return true to indicate that it succeeded
  // GIVEN the authenticate function will return true to indicate that it succeeded
  // GIVEn Spot has an arm
  ON_CALL(*spot_interface_ptr, createRobot).WillByDefault(Return(tl::expected<void, std::string>{}));
  ON_CALL(*spot_interface_ptr, authenticate).WillByDefault(Return(tl::expected<void, std::string>{}));
  ON_CALL(*spot_interface_ptr, hasArm).WillByDefault(Return(true));

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

  // GIVEN Spot has an arm
  ON_CALL(*spot_interface_ptr, hasArm).WillByDefault(Return(true));

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras + 1 hand camera = 18 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*spot_interface_ptr, getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 18)));
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

  // GIVEN Spot does not have an arm
  ON_CALL(*spot_interface_ptr, hasArm).WillByDefault(Return(false));

  // GIVEN the SpotImagePublisher was successfully initialized
  ASSERT_TRUE(image_publisher->initialize());

  {
    // THEN we send an image request to the Spot interface, and the request contains the expected number of cameras
    // (3 image types for 5 body cameras = 15 image requests)
    // THEN the images we received from the Spot interface are published
    // THEN the static transforms to the image frames are updated
    InSequence seq;
    EXPECT_CALL(*spot_interface_ptr, getImages(Property(&::bosdyn::api::GetImageRequest::image_requests_size, 15)));
    EXPECT_CALL(*publisher_interface_ptr, publish);
    EXPECT_CALL(*tf_interface_ptr, updateStaticTransforms);
  }

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}
}  // namespace spot_ros2::testing
