// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

namespace spot_ros2::conversions {

/**
 * @brief Convert a bosdyn protobuf position message to a ROS geometry point message
 *
 * @param proto Position from Spot
 * @param ros_msg Output ROS message
 */
void convertToRos(const ::bosdyn::api::Vec3& proto, geometry_msgs::msg::Point& ros_msg);

/**
 * @brief Convert a bosdyn protobuf translation message to a ROS geometry translation message
 *
 * @param proto Translation from Spot
 * @param ros_msg Output ROS message
 */
void convertToRos(const ::bosdyn::api::Vec3& proto, geometry_msgs::msg::Vector3& ros_msg);

/**
 * @brief Convert a bosdyn protobuf rotation message to a ROS geometry rotation message
 *
 * @param proto Rotation from Spot
 * @param ros_msg Output ROS message
 */
void convertToRos(const ::bosdyn::api::Quaternion& proto, geometry_msgs::msg::Quaternion& ros_msg);

/**
 * @brief Convert SE3Pose messages to ROS TransformStamped messages
 *
 * @param transform Spot SE3Pose proto message
 * @param parent_frame transform's parent frame
 * @param child_frame transform's child frame
 * @param tf_time transform's timestamp
 */

geometry_msgs::msg::TransformStamped toTransformStamped(const ::bosdyn::api::SE3Pose& transform,
                                                        const std::string& parent_frame, const std::string& child_frame,
                                                        const builtin_interfaces::msg::Time& tf_time);

}  // namespace spot_ros2::conversions
