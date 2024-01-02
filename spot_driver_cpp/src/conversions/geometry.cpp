
// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/conversions/geometry.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace spot_ros2::conversions{

geometry_msgs::msg::Point toRosPoint(const ::bosdyn::api::Vec3& msg){
  geometry_msgs::msg::Point out;
  
  out.x = msg.x();
  out.y = msg.y();
  out.z = msg.z();
  
  return out;
}

geometry_msgs::msg::Vector3 toRosTranslation(const ::bosdyn::api::Vec3& msg){
  geometry_msgs::msg::Vector3 out;
  
  out.x = msg.x();
  out.y = msg.y();
  out.z = msg.z();
  
  return out;
}

geometry_msgs::msg::Quaternion toRosRotation(const ::bosdyn::api::Quaternion& msg){
  geometry_msgs::msg::Quaternion out;
  
  out.x = msg.x();
  out.y = msg.y();
  out.z = msg.z();
  out.w = msg.w();
  
  return out;
}

geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose& transform,
                                                        const std::string& parent_frame, const std::string& child_frame,
                                                        const builtin_interfaces::msg::Time& tf_time) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = tf_time;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  tf.transform.translation = toRosTranslation(transform.position());
  tf.transform.rotation = toRosRotation(transform.rotation());

  return tf;
}
}  // namespace spot_ros2::conversions
