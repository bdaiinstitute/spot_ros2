// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/interfaces/image_client_interface.hpp>

#include <string>

namespace spot_ros2::test {
class MockImageClient : public ImageClientInterface {
 public:
  MOCK_METHOD((tl::expected<GetImagesResult, std::string>), getImages, (::bosdyn::api::GetImageRequest, bool, bool),
              (override));
};
}  // namespace spot_ros2::test
