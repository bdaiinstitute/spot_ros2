// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/api/image_client_api.hpp>
#include <spot_driver_cpp/api/spot_api.hpp>

namespace spot_ros2::test {
class MockSpotApi : public SpotApi {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), createRobot, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<void, std::string>), authenticate, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<bool, std::string>), hasArm, (), (const, override));
  MOCK_METHOD(std::shared_ptr<ImageClientApi>, image_client_api, (), (const, override));
};
}  // namespace spot_ros2::test