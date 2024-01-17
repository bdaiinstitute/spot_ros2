// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <bosdyn/api/robot_state.pb.h>
#include <google/protobuf/duration.pb.h>
#include <memory>
#include <spot_driver_cpp/conversions/robot_state.hpp>
#include <spot_msgs/msg/detail/battery_state__struct.hpp>
#include <spot_msgs/msg/detail/battery_state_array__struct.hpp>
#include <spot_msgs/msg/detail/wi_fi_state__struct.hpp>
#include "gmock/gmock-matchers.h"

namespace {
::bosdyn::api::BatteryState createBatteryState(const std::string& id, uint32_t percentage, uint32_t current,
                                               uint32_t voltage, double temperature,
                                               ::bosdyn::api::BatteryState_Status status) {
  ::bosdyn::api::BatteryState out;

  out.set_identifier(id);
  out.set_status(status);
  out.add_temperatures(temperature);
  out.mutable_charge_percentage()->set_value(percentage);
  out.mutable_current()->set_value(current);
  out.mutable_voltage()->set_value(voltage);

  return out;
}

::bosdyn::api::WiFiState createWifiState(::bosdyn::api::WiFiState_Mode mode, const std::string& essid) {
  ::bosdyn::api::WiFiState out;

  out.set_current_mode(mode);
  *out.mutable_essid() = essid;

  return out;
}
}  // namespace

namespace spot_ros2::conversions::test {

TEST(RobotStateConversions, TestGetBatteryStates) {
  // GIVEN a clock skew and a RobotState containing a BatteryState, a WifiState, FootState, and an EstopState
  google::protobuf::Duration clock_skew;
  ::bosdyn::api::RobotState robot_state;
  auto battery_state = robot_state.add_battery_states();
  battery_state->CopyFrom(createBatteryState(
      "test_battery", 50, 10, 12, 80.0, ::bosdyn::api::BatteryState_Status::BatteryState_Status_STATUS_DISCHARGING));

  // WHEN we create a BatteryStateArray ROS message from the RobotState
  const spot_msgs::msg::BatteryStateArray out = getBatteryStates(robot_state, clock_skew);

  // THEN the ROS message contains the same values as were set in the input RobotState
  ASSERT_THAT(out.battery_states, testing::SizeIs(1));
  const auto& first_battery_state = out.battery_states.at(0);
  EXPECT_THAT(first_battery_state.identifier, testing::StrEq("test_battery"));
  EXPECT_THAT(first_battery_state.charge_percentage, testing::Eq(50));
  EXPECT_THAT(first_battery_state.current, testing::Eq(10));
  EXPECT_THAT(first_battery_state.voltage, testing::Eq(12));
  ASSERT_THAT(first_battery_state.temperatures, testing::SizeIs(1));
  EXPECT_THAT(first_battery_state.temperatures.at(0), testing::DoubleEq(80.0));
}

// TEST(RobotStateConversions, TestGetJointStatesHasKinematicState)
// {

// }

// TEST(RobotStateConversions, TestGetJointStatesNoKinematicState)
// {

// }

TEST(RobotStateConversions, TestGetWifiState) {
  // GIVEN a RobotState which contains a valid CommsState field
  ::bosdyn::api::RobotState robot_state;
  auto comms_state = robot_state.add_comms_states();
  comms_state->mutable_wifi_state()->CopyFrom(
      createWifiState(::bosdyn::api::WiFiState_Mode::WiFiState_Mode_MODE_CLIENT, "some_value"));

  // WHEN we create a WiFiState ROS message from the RobotState
  const spot_msgs::msg::WiFiState out = getWifiState(robot_state);

  // THEN the ROS message contains the same values for the ESSID and mode fields as were set in the input RobotState
  EXPECT_THAT(out.current_mode, testing::Eq(spot_msgs::msg::WiFiState::MODE_CLIENT));
  EXPECT_THAT(out.essid, testing::StrEq("some_value"));
}

// TEST(RobotStateConversions, TestGetTf_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetTf_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetOdomTwist_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetOdomTwist_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetOdom_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetOdom_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetPowerState_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetPowerState_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetSystemFaultState_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetSystemFaultState_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetManipulatorState_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetManipulatorState_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetEndEffectorForce_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetEndEffectorForce_no_kinematic_state) {}

// TEST(RobotStateConversions, TestGetBehaviorFaultState_has_kinematic_state) {}
// TEST(RobotStateConversions, TestGetBehaviorFaultState_no_kinematic_state) {}
}  // namespace spot_ros2::conversions::test
