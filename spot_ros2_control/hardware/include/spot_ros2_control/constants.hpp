// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

// Helper constants

#pragma once

#include <map>
#include <string>
#include <vector>

namespace spot_ros2_control {
namespace constants {

// From https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#bosdyn-api-spot-JointIndex
// This is the order the commands need to be sent in
std::map<std::string, int> spot_joint_index_map = {
    {"front_left_hip_x", 0},  {"front_left_hip_y", 1}, {"front_left_knee", 2},   {"front_right_hip_x", 3},
    {"front_right_hip_y", 4}, {"front_right_knee", 5}, {"rear_left_hip_x", 6},   {"rear_left_hip_y", 7},
    {"rear_left_knee", 8},    {"rear_right_hip_x", 9}, {"rear_right_hip_y", 10}, {"rear_right_knee", 11},
    {"arm_sh0", 12},          {"arm_sh1", 13},         {"arm_el0", 14},          {"arm_el1", 15},
    {"arm_wr0", 16},          {"arm_wr1", 17},         {"arm_f1x", 18},
};
} // namespace constants
} // namespace spot_ros2_control