// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/spot/inverse_kinematics.pb.h>

#include <bosdyn_msgs/msg/inverse_kinematics_request.hpp>
#include <bosdyn_msgs/msg/inverse_kinematics_response.hpp>

#include <spot_driver_cpp/kinematic_service.hpp>

namespace spot_ros2::kinematic_utils {

void convert_time_to_proto(const builtin_interfaces::msg::Time& ros_msg, google::protobuf::Timestamp& proto);

void convert_request_header_to_proto(const bosdyn_msgs::msg::RequestHeader& ros_msg, bosdyn::api::RequestHeader& proto);

void convert_inverse_kinematics_request_to_proto(const bosdyn_msgs::msg::InverseKinematicsRequest& ros_msg,
                                                 bosdyn::api::spot::InverseKinematicsRequest& proto);

void convert_proto_to_inverse_kinematics_response(const bosdyn::api::spot::InverseKinematicsResponse& proto,
                                                  bosdyn_msgs::msg::InverseKinematicsResponse& ros_msg);

}  // namespace spot_ros2::kinematic_utils