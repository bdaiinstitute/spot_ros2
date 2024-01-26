// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/api/kinematic_api.hpp>
#include <spot_driver_cpp/api/spot_api.hpp>
#include <spot_driver_cpp/interfaces/image_client_interface.hpp>

#include <memory>
#include <string>

namespace spot_ros2::test {
class MockSpotApi : public SpotApi {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), createRobot, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<void, std::string>), authenticate, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<bool, std::string>), hasArm, (), (const, override));
  MOCK_METHOD(std::shared_ptr<KinematicApi>, kinematicApi, (), (const, override));
  MOCK_METHOD(std::shared_ptr<ImageClientInterface>, image_client_interface, (), (const, override));
};
}  // namespace spot_ros2::test
