// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver_cpp/interfaces/middleware_interface_base.hpp>
#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>
#include <spot_driver_cpp/interfaces/publisher_interface_base.hpp>
#include <spot_driver_cpp/interfaces/tf_interface_base.hpp>
#include <spot_driver_cpp/interfaces/timer_interface_base.hpp>

#include <memory>
#include <optional>
#include <tl_expected/expected.hpp>

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

class MockMiddlewareInterface : public MiddlewareInterface {
 public:
  MOCK_METHOD(ParameterInterfaceBase*, parameter_interface, (), (override));
  MOCK_METHOD(LoggerInterfaceBase*, logger_interface, (), (override));
  MOCK_METHOD(PublisherInterfaceBase*, publisher_interface, (), (override));
  MOCK_METHOD(TfInterfaceBase*, tf_interface, (), (override));
  MOCK_METHOD(TimerInterfaceBase*, timer_interface, (), (override));
};
}