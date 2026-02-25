# Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

find_package(ament_cmake_pytest REQUIRED)

ament_add_pytest_test(spot_ros2_control_pytest ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 60)
