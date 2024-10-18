// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string>
#include <unordered_map>
#include <vector>

#pragma once

namespace spot_hardware_interface {

// Gain values https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/constants.hpp
// This will be handled via a parameter in the future so there is the option to change them, for now they are hardcoded

// kp and kd gains for a robot without an arm
const std::vector<float> no_arm_kp = {624, 936, 286, 624, 936, 286, 624, 936, 286, 624, 936, 286};
const std::vector<float> no_arm_kd = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04};
// kp and kd gains for a robot with an arm
const std::vector<float> arm_kp = {624, 936, 286,  624, 936, 286, 624, 936, 286, 624,
                                   936, 286, 1020, 255, 204, 102, 102, 102, 16.0};
const std::vector<float> arm_kd = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20,
                                   5.20, 2.04, 10.2, 15.3, 10.2, 2.04, 2.04, 2.04, 0.32};

/// @brief Number of joints we expect if the robot has an arm
inline constexpr int kNjointsArm = 19;
/// @brief Number of joints we expect if the robot has no arm
inline constexpr int kNjointsNoArm = 12;

}  // namespace spot_hardware_interface
