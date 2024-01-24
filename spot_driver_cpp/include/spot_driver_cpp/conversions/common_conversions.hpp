// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/wrappers.pb.h>

#include <bosdyn_msgs/msg/arm_joint_position.hpp>
#include <bosdyn_msgs/msg/common_error.hpp>
#include <bosdyn_msgs/msg/quaternion.hpp>
#include <bosdyn_msgs/msg/request_header.hpp>
#include <bosdyn_msgs/msg/response_header.hpp>

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

void convertBuiltinInterfacesTimeToProto(const builtin_interfaces::msg::Time& ros_msg,
                                         google::protobuf::Timestamp& proto);

void convertBosdynMsgsRequestHeaderToProto(const bosdyn_msgs::msg::RequestHeader& ros_msg,
                                           bosdyn::api::RequestHeader& proto);

void convertGeometryMsgsVector3ToProto(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3& proto);

void convertGeometryMsgsPointToProto(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3& proto);

void convertGeometryMsgsQuaternionToProto(const geometry_msgs::msg::Quaternion& ros_msg,
                                          bosdyn::api::Quaternion& proto);

void convertGeometryMsgsPoseToProto(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE3Pose& proto);

void convertfloat64ToProto(const double ros_msg, google::protobuf::DoubleValue& proto);

void convertBosdynMsgsArmJointPositionToProto(const bosdyn_msgs::msg::ArmJointPosition& ros_msg,
                                              bosdyn::api::ArmJointPosition& proto);

///////////////////////////////////////////////////////////////////////////////
// Protobuf to ROS.

void convertProtoToBosdynMsgsRequestHeader(const bosdyn::api::RequestHeader& proto,
                                           bosdyn_msgs::msg::RequestHeader& ros_msg);

void convertProtoToBosdynMsgsCommonError(const bosdyn::api::CommonError& proto, bosdyn_msgs::msg::CommonError& ros_msg);

void convertProtoToBosdynMsgsResponseHeader(const bosdyn::api::ResponseHeader& proto,
                                            bosdyn_msgs::msg::ResponseHeader& ros_msg);

void convertProtoToBuiltinInterfacesTime(const google::protobuf::Timestamp& proto,
                                         builtin_interfaces::msg::Time& ros_msg);

void convertProtoToGeometryMsgsVector3(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Vector3& ros_msg);

void convertProtoToGeometryMsgsVector3(const bosdyn::api::Vec3& proto, geometry_msgs::msg::Point& ros_msg);

void convertProtoToGeometryMsgsQuaternion(const bosdyn::api::Quaternion& proto,
                                          geometry_msgs::msg::Quaternion& ros_msg);

void convertProtoToGeometryMsgsPose(const bosdyn::api::SE3Pose& proto, geometry_msgs::msg::Pose& ros_msg);

void convertProtoToGeometryMsgsTwist(const bosdyn::api::SE3Velocity& proto, geometry_msgs::msg::Twist& ros_msg);

}  // namespace spot_ros2::common_conversions
