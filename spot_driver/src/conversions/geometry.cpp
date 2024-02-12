
// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <geometry_msgs/msg/point.hpp>
#include <spot_driver/conversions/geometry.hpp>

namespace spot_ros2::conversions {

void convertToRos(const ::bosdyn::api::Vec3& proto, geometry_msgs::msg::Point& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convertToRos(const ::bosdyn::api::Vec3& proto, geometry_msgs::msg::Vector3& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
}

void convertToRos(const ::bosdyn::api::Quaternion& proto, geometry_msgs::msg::Quaternion& ros_msg) {
  ros_msg.x = proto.x();
  ros_msg.y = proto.y();
  ros_msg.z = proto.z();
  ros_msg.w = proto.w();
}

geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose& transform,
                                                        const std::string& parent_frame, const std::string& child_frame,
                                                        const builtin_interfaces::msg::Time& tf_time) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = tf_time;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  convertToRos(transform.position(), tf.transform.translation);
  convertToRos(transform.rotation(), tf.transform.rotation);

  return tf;
}
}  // namespace spot_ros2::conversions
