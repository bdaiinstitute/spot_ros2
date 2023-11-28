// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/wrappers.pb.h>

#include <bosdyn_msgs/msg/arm_joint_position.hpp>
#include <bosdyn_msgs/msg/quaternion.hpp>
#include <bosdyn_msgs/msg/request_header.hpp>

#include <bosdyn/api/arm_command.pb.h>
#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/header.pb.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace spot_ros2::common_conversions {

///////////////////////////////////////////////////////////////////////////////
// ROS to Protobuf.

void convert_builtin_interfaces_time_to_proto(const builtin_interfaces::msg::Time& ros_msg,
                                              google::protobuf::Timestamp& proto);

void convert_bosdyn_msgs_request_header_to_proto(const bosdyn_msgs::msg::RequestHeader& ros_msg,
                                                 bosdyn::api::RequestHeader& proto);

void convert_geometry_msgs_vector3_to_proto(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3& proto);

void convert_geometry_msgs_point_to_proto(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3& proto);

void convert_geometry_msgs_quaternion_to_proto(const geometry_msgs::msg::Quaternion& ros_msg,
                                               bosdyn::api::Quaternion& proto);

void convert_geometry_msgs_pose_to_proto(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE3Pose& proto);

void convert_float64_to_proto(const double ros_msg, google::protobuf::DoubleValue& proto);

void convert_bosdyn_msgs_arm_joint_position_to_proto(const bosdyn_msgs::msg::ArmJointPosition& ros_msg,
                                                     bosdyn::api::ArmJointPosition& proto);

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convert_proto_to_builtin_interfaces_time(const google::protobuf::Timestamp& proto,
                                              builtin_interfaces::msg::Time& ros_msg);

void convert_proto_to_geometry_msgs_vector3(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Vector3& ros_msg);

void convert_proto_to_geometry_msgs_vector3(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Point& ros_msg);

void convert_proto_to_geometry_msgs_quaternion(const bosdyn::api::Quaternion& proto,
                                               geometry_msgs::msg::Quaternion& ros_msg);

void convert_proto_to_geometry_msgs_pose(const bosdyn::api::SE3Pose& proto, geometry_msgs::msg::Pose& ros_msg);

void convert_proto_to_geometry_msgs_twist(const bosdyn::api::SE3Velocity& proto, geometry_msgs::msg::Twist& ros_msg);

}  // namespace spot_ros2::common_conversions