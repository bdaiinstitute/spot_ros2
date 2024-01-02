// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/conversions/geometry.hpp>

namespace {

struct GeometryParams{
  ::bosdyn::api::SE3Pose transform;
  std::string parent_frame;
  std::string child_frame;
  builtin_interfaces::msg::Time tf_time;
}

}

namespace spot_ros2::conversions::test {

class GeometryConversionTestFixture : public ::testing::TestWithParams<GeometryParams> {}

const auto kTransformStampedTestParameters = testing::Values(
  GeometryParams{}
)

TEST_P(GeometryConversionTestFixture, toTransformStampedTests){
  transform, parent_frame, child_frame, tf_time = GetParam();
}

INSTANTIATE_TEST_SUITE_P(GeometryParamterizedTests,
                         GeometryConversionTestFixture,
                         testing::Values("meeny", "miny", "moe"));

} // namespace spot_ros2::conversions::test
