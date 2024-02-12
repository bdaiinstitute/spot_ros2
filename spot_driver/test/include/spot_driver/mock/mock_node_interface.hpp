// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>
#include <memory>
#include <spot_driver/interfaces/node_interface_base.hpp>

namespace spot_ros2::test {
class MockNodeInterface : public NodeInterfaceBase {
 public:
  MOCK_METHOD(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>, getNodeBaseInterface, (), (override));
};
}  // namespace spot_ros2::test
