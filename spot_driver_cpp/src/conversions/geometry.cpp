
// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/conversions/geometry.hpp>

namespace spot_ros2::conversions{

geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose& transform,
                                                        const std::string& parent_frame, const std::string& child_frame,
                                                        const builtin_interfaces::msg::Time& tf_time){
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = tf_time;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = transform.position().x();
  tf.transform.translation.y = transform.position().y();
  tf.transform.translation.z = transform.position().z();

  tf.transform.rotation.x = transform.rotation().x();
  tf.transform.rotation.y = transform.rotation().y();
  tf.transform.rotation.z = transform.rotation().z();
  tf.transform.rotation.w = transform.rotation().w();

  return tf;
}
} // namespace spot_ros2::conversions
