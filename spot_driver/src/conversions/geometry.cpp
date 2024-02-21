
// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/conversions/geometry.hpp>

#include <spot_driver/conversions/common_conversions.hpp>

namespace spot_ros2::conversions {
geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose& transform,
                                                        const std::string& parent_frame, const std::string& child_frame,
                                                        const builtin_interfaces::msg::Time& tf_time) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = tf_time;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  common_conversions::convertToRos(transform.position(), tf.transform.translation);
  common_conversions::convertToRos(transform.rotation(), tf.transform.rotation);

  return tf;
}
}  // namespace spot_ros2::conversions
