#pragma once

#include <gmock/gmock.h>
#include <spot_driver/robot_state/state_publisher.hpp>
#include <spot_driver/types.hpp>

namespace spot_ros2::test {
class MockStateMiddlewareHandle : public StatePublisher::MiddlewareHandle {
 public:
  MOCK_METHOD(void, publishRobotState, (const RobotStateMessages& robot_state), (override));
};
}  // namespace spot_ros2::test
