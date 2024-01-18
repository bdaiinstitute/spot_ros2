// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/robot_state.pb.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <memory>
#include <spot_driver_cpp/conversions/robot_state.hpp>
#include <spot_msgs/msg/detail/battery_state__struct.hpp>
#include <spot_msgs/msg/detail/battery_state_array__struct.hpp>
#include <spot_msgs/msg/detail/wi_fi_state__struct.hpp>
#include "gmock/gmock-matchers.h"
#include "gmock/gmock-more-matchers.h"

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

TEST(RobotStateConversions, TestGetFootState) {}

TEST(RobotStateConversions, TestGetEStopStates) {}

TEST(RobotStateConversions, TestGetJointStates) {}

TEST(RobotStateConversions, TestGetTf) {}

TEST(RobotStateConversions, TestGetOdomTwist) {}

TEST(RobotStateConversions, TestGetOdom) {}

TEST(RobotStateConversions, TestGetPowerState) {}

TEST(RobotStateConversions, TestGetSystemFaultState) {}

TEST(RobotStateConversions, TestGetManipulatorState) {}

TEST(RobotStateConversions, TestGetEndEffectorForce) {
  // GIVEN nominal timestamps and nonzero clock skew
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(10);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  const auto prefix = "prefix/";

  // GIVEN a RobotState containing the estimated end effector force
  ::bosdyn::api::RobotState robot_state;
  ::bosdyn::api::Vec3 force;
  force.set_x(1.0);
  force.set_y(2.0);
  force.set_z(3.0);
  robot_state.mutable_manipulator_state()->mutable_estimated_end_effector_force_in_hand()->CopyFrom(force);
  robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp()->CopyFrom(timestamp);

  // WHEN we create a Vector3Stamped ROS message to represent the estimated end effector force
  const auto out = getEndEffectorForce(robot_state, clock_skew, prefix);

  // THEN the fields match
  ASSERT_THAT(out.has_value(), testing::IsTrue());
  EXPECT_THAT(out->header.frame_id, testing::StrEq("prefix/hand"));
  EXPECT_THAT(out->header.stamp.sec, testing::Eq(9));
  EXPECT_THAT(out->vector.x, testing::Eq(force.x()));
  EXPECT_THAT(out->vector.y, testing::Eq(force.y()));
  EXPECT_THAT(out->vector.z, testing::Eq(force.z()));
}

TEST(RobotStateConversions, TestGetEndEffectorForceNoEndEffectorForce) {
  // GIVEN an empty RobotState
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  const auto prefix = "prefix/";
  ::bosdyn::api::RobotState robot_state;

  // WHEN we call getEndEffectorForce()
  const auto out = getEndEffectorForce(robot_state, clock_skew, prefix);

  // THEN the conversion does not succeed
  EXPECT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetBehaviorFaultState) {
  // GIVEN nominal timestamps and nonzero clock skew
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(10);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState containing two faults
  ::bosdyn::api::RobotState robot_state;
  auto fault_state_first = robot_state.mutable_behavior_fault_state()->add_faults();
  fault_state_first->set_behavior_fault_id(11);
  fault_state_first->set_status(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_CLEARABLE);
  fault_state_first->set_cause(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_HARDWARE);
  fault_state_first->mutable_onset_timestamp()->CopyFrom(timestamp);
  auto fault_state_second = robot_state.mutable_behavior_fault_state()->add_faults();
  fault_state_second->set_behavior_fault_id(12);
  fault_state_second->set_status(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_UNCLEARABLE);
  fault_state_second->set_cause(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_FALL);
  fault_state_second->mutable_onset_timestamp()->CopyFrom(timestamp);

  // WHEN we create a BehaviorFaultState ROS message from the robot state
  const auto out = getBehaviorFaultState(robot_state, clock_skew);

  // THEN the output optional contains a message
  ASSERT_THAT(out.has_value(), testing::IsTrue());
  // THEN the message contains two faults
  ASSERT_THAT(out->faults, testing::SizeIs(2));
  // THEN the first fault matches the first one added to the RobotState, and the clock skew is applied correctly
  const auto first_fault = out->faults.at(0);
  EXPECT_THAT(first_fault.behavior_fault_id, testing::Eq(11));
  EXPECT_THAT(first_fault.status,
              testing::Eq(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_CLEARABLE));
  EXPECT_THAT(first_fault.cause, testing::Eq(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_HARDWARE));
  EXPECT_THAT(first_fault.header.stamp.sec, testing::Eq(9));
  EXPECT_THAT(first_fault.header.stamp.nanosec, testing::Eq(0));
  // THEN the second fault matches the second one added to the RobotState, and the clock skew is applied correctly
  const auto second_fault = out->faults.at(1);
  EXPECT_THAT(second_fault.behavior_fault_id, testing::Eq(12));
  EXPECT_THAT(second_fault.status,
              testing::Eq(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_UNCLEARABLE));
  EXPECT_THAT(second_fault.cause, testing::Eq(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_FALL));
  EXPECT_THAT(second_fault.header.stamp.sec, testing::Eq(9));
  EXPECT_THAT(second_fault.header.stamp.nanosec, testing::Eq(0));
}

TEST(RobotStateConversions, TestGetBehaviorFaultStateNoFaults) {
  // GIVEN a robot state that does not contain any behavior fault states
  google::protobuf::Duration clock_skew;
  ::bosdyn::api::RobotState robot_state;

  // WHEN we create a BehaviorFaultState ROS message from the robot state
  const auto out = getBehaviorFaultState(robot_state, clock_skew);

  // THEN the optional which wraps the output ROS message is set to nullopt
  EXPECT_THAT(out.has_value(), testing::IsFalse());
}

// TEST(RobotStateConversions, TestGetJointStatesHasKinematicState)
// {

// }

// TEST(RobotStateConversions, TestGetJointStatesNoKinematicState)
// {

// }

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
