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

#pragma once

namespace spot_hardware_interface {

/// @brief Number of joints we expect if the robot has no arm
inline constexpr int kNjointsNoArm = 12;

/// @brief Number of joints we expect if the robot has an arm
inline constexpr int kNjointsArm = 19;

}  // namespace spot_hardware_interface
