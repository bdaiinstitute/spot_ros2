// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

namespace spot_ros2::conversions {

geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose& transform,
                                                        const std::string& parent_frame, const std::string& child_frame,
                                                        const builtin_interfaces::msg::Time& tf_time);

}  // namespace spot_ros2::conversions
