// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/api/kinematic_api.hpp>
#include <spot_driver_cpp/interfaces/logger_interface_base.hpp>
#include <spot_driver_cpp/interfaces/parameter_interface_base.hpp>

namespace spot_ros2::test {
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

class MockKinematicApi : public KinematicApi {
 public:
  MOCK_METHOD((tl::expected<Result<InverseKinematicsResponse>, std::string>), get_solutions,
              (InverseKinematicsRequest & request), (override));
}
}  // namespace spot_ros2::test