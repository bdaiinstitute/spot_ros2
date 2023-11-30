// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/api/image_client_api.hpp>

#include <string>

namespace spot_ros2::test {
class MockImageClientApi : public ImageClientApi {
 public:
  MOCK_METHOD((tl::expected<GetImagesResult, std::string>), getImages, (::bosdyn::api::GetImageRequest), (override));
};
}  // namespace spot_ros2::test
