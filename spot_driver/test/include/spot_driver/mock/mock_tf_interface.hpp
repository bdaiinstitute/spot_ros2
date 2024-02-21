// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/interfaces/tf_interface_base.hpp>

#include <string>
#include <vector>

namespace spot_ros2::test {
class MockTfInterface : public TfInterfaceBase {
 public:
  MOCK_METHOD(void, updateStaticTransforms, (const std::vector<geometry_msgs::msg::TransformStamped>& transforms),
              (override));
};
}  // namespace spot_ros2::test
