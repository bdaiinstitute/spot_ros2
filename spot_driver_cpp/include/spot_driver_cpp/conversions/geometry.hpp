// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved. 

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <bosdyn/math/frame_helpers.h>
#include <bosdyn/api/geometry.pb.h>
#include <builtin_interfaces/msg/time.hpp>

geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose &transform, const std::string &parent_frame, 
                                                        const std::string &child_frame, const builtin_interfaces::msg::Time &tf_time){
    auto tf = geometry_msgs::msg::TransformStamped();
    tf.header.stamp = tf_time;
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;
    
  const auto& position = transform.position();
    tf.transform.translation =
        geometry_msgs::build<geometry_msgs::msg::Vector3>().x(position.x()).y(position.y()).z(position.z());

    const auto& rotation = transform.rotation();
    tf.transform.rotation = geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                       .x(rotation.x())
                                       .y(rotation.y())
                                       .z(rotation.z())
                                       .w(rotation.w());
    
    return tf;
}