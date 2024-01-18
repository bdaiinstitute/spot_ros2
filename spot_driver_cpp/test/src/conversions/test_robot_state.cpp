// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/robot_state.pb.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <bosdyn_msgs/msg/manipulator_state_carry_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iterator>
#include <memory>
#include <spot_driver_cpp/conversions/robot_state.hpp>
#include <spot_msgs/msg/battery_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include "gmock/gmock-matchers.h"
#include "gmock/gmock-more-matchers.h"

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

void appendSystemFault(::bosdyn::api::SystemFaultState* mutable_fault_state,
                       const google::protobuf::Timestamp& onset_timestamp, const google::protobuf::Duration& duration,
                       const std::string& name, const int32_t code, const uint64_t uid,
                       const std::string& error_message, const std::vector<std::string>& attributes,
                       const ::bosdyn::api::SystemFault_Severity& severity) {
  auto fault = mutable_fault_state->add_faults();
  fault->mutable_onset_timestamp()->CopyFrom(onset_timestamp);
  fault->mutable_duration()->CopyFrom(duration);
  fault->set_name(name);
  fault->set_code(code);
  fault->set_uid(uid);
  fault->set_error_message(error_message);
  *fault->mutable_attributes() = {attributes.cbegin(), attributes.cend()};
  fault->set_severity(severity);
}

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

TEST(RobotStateConversions, TestGetPowerState) {
  // GIVEN a RobotState that contains a fully-populated power state
  ::bosdyn::api::RobotState robot_state;
  robot_state.mutable_power_state()->set_robot_power_state(
      ::bosdyn::api::PowerState_RobotPowerState::PowerState_RobotPowerState_ROBOT_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_motor_power_state(
      ::bosdyn::api::PowerState_MotorPowerState::PowerState_MotorPowerState_MOTOR_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_shore_power_state(
      ::bosdyn::api::PowerState_ShorePowerState::PowerState_ShorePowerState_SHORE_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_payload_ports_power_state(
      ::bosdyn::api::PowerState_PayloadPortsPowerState::PowerState_PayloadPortsPowerState_PAYLOAD_PORTS_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_wifi_radio_power_state(
      ::bosdyn::api::PowerState_WifiRadioPowerState::PowerState_WifiRadioPowerState_WIFI_RADIO_POWER_STATE_ON);
  robot_state.mutable_power_state()->mutable_locomotion_charge_percentage()->set_value(75.0);
  google::protobuf::Duration estimated_runtime;
  estimated_runtime.set_seconds(255);
  estimated_runtime.set_nanos(0);
  robot_state.mutable_power_state()->mutable_locomotion_estimated_runtime()->CopyFrom(estimated_runtime);
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(60);
  robot_state.mutable_power_state()->mutable_timestamp()->CopyFrom(timestamp);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a PowerState ROS message
  const auto out = getPowerState(robot_state, clock_skew);

  // THEN a message is output
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN the fields in the output message match their corresponding inputs
  EXPECT_THAT(out->motor_power_state, testing::Eq(spot_msgs::msg::PowerState::STATE_ON));
  EXPECT_THAT(out->shore_power_state, testing::Eq(spot_msgs::msg::PowerState::STATE_ON_SHORE_POWER));
  EXPECT_THAT(out->locomotion_charge_percentage, testing::DoubleEq(75.0));
  EXPECT_THAT(out->locomotion_estimated_runtime,
              testing::AllOf(testing::Field("sec", &builtin_interfaces::msg::Duration::sec, testing::Eq(255)),
                             testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(0))));
}

TEST(RobotStateConversions, TestGetPowerStateNoPowerState) {
  // GIVEN a RobotState that does not contain a power state
  ::bosdyn::api::RobotState robot_state;

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a PowerState ROS message
  const auto out = getPowerState(robot_state, clock_skew);

  // THEN no message is output
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetSystemFaultState) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN the robot state contains two system faults, which occur at different timestamps and contain different data
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp1;
  timestamp1.set_seconds(60);
  google::protobuf::Duration duration1;
  duration1.set_seconds(15);
  duration1.set_nanos(0);
  appendSystemFault(robot_state.mutable_system_fault_state(), timestamp1, duration1, "fault1", 19, 3, "battery is low",
                    {"robot", "battery"}, ::bosdyn::api::SystemFault_Severity::SystemFault_Severity_SEVERITY_WARN);

  google::protobuf::Timestamp timestamp2;
  timestamp2.set_seconds(75);
  google::protobuf::Duration duration2;
  duration2.set_seconds(0);
  duration2.set_nanos(0);
  appendSystemFault(robot_state.mutable_system_fault_state(), timestamp2, duration2, "fault2", 55, 9,
                    "robot has departed from this plane of reality", {"robot"},
                    ::bosdyn::api::SystemFault_Severity::SystemFault_Severity_SEVERITY_CRITICAL);

  // WHEN we create a SystemFaultState ROS message from the Robot State
  auto out = getSystemFaultState(robot_state, clock_skew);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN the output message contains two faults
  // THEN each of the output faults contains fields that match the input
  // THEN the clock skew is correctly applied
  // TODO(schornakj): revise gross matcher so it gives more useful info on test failure
  using SystemFault = spot_msgs::msg::SystemFault;
  EXPECT_THAT(
      out->faults,
      testing::UnorderedElementsAre(
          testing::AllOf(
              testing::Field(
                  "header", &SystemFault::header,
                  testing::AllOf(testing::Field(
                      "stamp", &std_msgs::msg::Header::stamp,
                      testing::AllOf(
                          testing::Field("sec", &builtin_interfaces::msg::Time::sec, testing::Eq(59)),
                          testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec, testing::Eq(0)))))),
              testing::Field(
                  "duration", &SystemFault::duration,
                  testing::AllOf(
                      testing::Field("sec", &builtin_interfaces::msg::Duration::sec, testing::Eq(15)),
                      testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(0)))),
              testing::Field("name", &SystemFault::name, testing::StrEq("fault1")),
              testing::Field("code", &SystemFault::code, testing::Eq(19)),
              testing::Field("uid", &SystemFault::uid, testing::Eq(3)),
              testing::Field("error_message", &SystemFault::error_message, testing::StrEq("battery is low")),
              testing::Field("attributes", &SystemFault::attributes,
                             testing::UnorderedElementsAre(testing::StrEq("robot"), testing::StrEq("battery")))),
          testing::AllOf(
              testing::Field(
                  "header", &SystemFault::header,
                  testing::AllOf(testing::Field(
                      "stamp", &std_msgs::msg::Header::stamp,
                      testing::AllOf(
                          testing::Field("sec", &builtin_interfaces::msg::Time::sec, testing::Eq(74)),
                          testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec, testing::Eq(0)))))),
              testing::Field(
                  "duration", &SystemFault::duration,
                  testing::AllOf(
                      testing::Field("sec", &builtin_interfaces::msg::Duration::sec, testing::Eq(0)),
                      testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(0)))),
              testing::Field("name", &SystemFault::name, testing::StrEq("fault2")),
              testing::Field("code", &SystemFault::code, testing::Eq(55)),
              testing::Field("uid", &SystemFault::uid, testing::Eq(9)),
              testing::Field("error_message", &SystemFault::error_message,
                             testing::StrEq("robot has departed from this plane of reality")),
              testing::Field("attributes", &SystemFault::attributes,
                             testing::UnorderedElementsAre(testing::StrEq("robot"))))));
}

TEST(RobotStateConversions, TestGetSystemFaultStateNoFault) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that does not contain any system faults
  ::bosdyn::api::RobotState robot_state;

  // WHEN we create a SystemFaultState ROS message from the Robot State
  auto out = getSystemFaultState(robot_state, clock_skew);

  // THEN no ROS message is output
  EXPECT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetManipulatorState) {
  // GIVEN a RobotState which has all manipulator state fields populated
  ::bosdyn::api::RobotState robot_state;
  robot_state.mutable_manipulator_state()->set_gripper_open_percentage(50.0);
  robot_state.mutable_manipulator_state()->set_is_gripper_holding_item(true);
  ::bosdyn::api::Vec3 force_in_hand;
  force_in_hand.set_x(1.0);
  force_in_hand.set_y(2.0);
  force_in_hand.set_z(3.0);
  robot_state.mutable_manipulator_state()->mutable_estimated_end_effector_force_in_hand()->CopyFrom(force_in_hand);

  ::bosdyn::api::SE3Velocity velocity_hand_vision;
  velocity_hand_vision.mutable_linear()->set_x(1.0);
  velocity_hand_vision.mutable_linear()->set_y(2.0);
  velocity_hand_vision.mutable_linear()->set_z(3.0);
  velocity_hand_vision.mutable_angular()->set_x(4.0);
  velocity_hand_vision.mutable_angular()->set_y(5.0);
  velocity_hand_vision.mutable_angular()->set_z(6.0);
  robot_state.mutable_manipulator_state()->mutable_velocity_of_hand_in_vision()->CopyFrom(velocity_hand_vision);

  ::bosdyn::api::SE3Velocity velocity_hand_odom;
  velocity_hand_odom.mutable_linear()->set_x(1.0);
  velocity_hand_odom.mutable_linear()->set_y(2.0);
  velocity_hand_odom.mutable_linear()->set_z(3.0);
  velocity_hand_odom.mutable_angular()->set_x(4.0);
  velocity_hand_odom.mutable_angular()->set_y(5.0);
  velocity_hand_odom.mutable_angular()->set_z(6.0);
  robot_state.mutable_manipulator_state()->mutable_velocity_of_hand_in_odom()->CopyFrom(velocity_hand_odom);

  robot_state.mutable_manipulator_state()->set_carry_state(
      ::bosdyn::api::ManipulatorState_CarryState::ManipulatorState_CarryState_CARRY_STATE_NOT_CARRIABLE);
  robot_state.mutable_manipulator_state()->set_stow_state(
      ::bosdyn::api::ManipulatorState_StowState::ManipulatorState_StowState_STOWSTATE_DEPLOYED);

  // WHEN we use the RobotState to create a ManipulatorState ROS message
  const auto out = getManipulatorState(robot_state);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN all the output fields are set, and contain the same values as the inputs
  EXPECT_THAT(out->gripper_open_percentage, testing::Eq(50.0));
  EXPECT_THAT(out->is_gripper_holding_item, testing::IsTrue());
  EXPECT_THAT(out->carry_state.value,
              testing::Eq(bosdyn_msgs::msg::ManipulatorStateCarryState::CARRY_STATE_NOT_CARRIABLE));
  EXPECT_THAT(out->stow_state.value, testing::Eq(bosdyn_msgs::msg::ManipulatorStateStowState::STOWSTATE_DEPLOYED));

  using Vector3 = geometry_msgs::msg::Vector3;
  using Twist = geometry_msgs::msg::Twist;

  EXPECT_THAT(out->estimated_end_effector_force_in_hand_is_set, testing::IsTrue());
  EXPECT_THAT(out->estimated_end_effector_force_in_hand,
              testing::Field(&Vector3::x, testing::DoubleEq(force_in_hand.x())));
  EXPECT_THAT(out->estimated_end_effector_force_in_hand,
              testing::Field(&Vector3::y, testing::DoubleEq(force_in_hand.y())));
  EXPECT_THAT(out->estimated_end_effector_force_in_hand,
              testing::Field(&Vector3::z, testing::DoubleEq(force_in_hand.z())));

  EXPECT_THAT(out->velocity_of_hand_in_vision_is_set, testing::IsTrue());
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              testing::Field(&Twist::linear,
                             testing::Field(&Vector3::x, testing::DoubleEq(velocity_hand_vision.linear().x()))));
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              testing::Field(&Twist::linear,
                             testing::Field(&Vector3::y, testing::DoubleEq(velocity_hand_vision.linear().y()))));
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              testing::Field(&Twist::linear,
                             testing::Field(&Vector3::z, testing::DoubleEq(velocity_hand_vision.linear().z()))));
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              testing::Field(&Twist::angular,
                             testing::Field(&Vector3::x, testing::DoubleEq(velocity_hand_vision.angular().x()))));
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              testing::Field(&Twist::angular,
                             testing::Field(&Vector3::y, testing::DoubleEq(velocity_hand_vision.angular().y()))));
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              testing::Field(&Twist::angular,
                             testing::Field(&Vector3::z, testing::DoubleEq(velocity_hand_vision.angular().z()))));

  EXPECT_THAT(out->velocity_of_hand_in_odom_is_set, testing::IsTrue());
  EXPECT_THAT(
      out->velocity_of_hand_in_odom,
      testing::Field(&Twist::linear, testing::Field(&Vector3::x, testing::DoubleEq(velocity_hand_odom.linear().x()))));
  EXPECT_THAT(
      out->velocity_of_hand_in_odom,
      testing::Field(&Twist::linear, testing::Field(&Vector3::y, testing::DoubleEq(velocity_hand_odom.linear().y()))));
  EXPECT_THAT(
      out->velocity_of_hand_in_odom,
      testing::Field(&Twist::linear, testing::Field(&Vector3::z, testing::DoubleEq(velocity_hand_odom.linear().z()))));
  EXPECT_THAT(out->velocity_of_hand_in_odom,
              testing::Field(&Twist::angular,
                             testing::Field(&Vector3::x, testing::DoubleEq(velocity_hand_odom.angular().x()))));
  EXPECT_THAT(out->velocity_of_hand_in_odom,
              testing::Field(&Twist::angular,
                             testing::Field(&Vector3::y, testing::DoubleEq(velocity_hand_odom.angular().y()))));
  EXPECT_THAT(out->velocity_of_hand_in_odom,
              testing::Field(&Twist::angular,
                             testing::Field(&Vector3::z, testing::DoubleEq(velocity_hand_odom.angular().z()))));
}

TEST(RobotStateConversions, TestGetManipulatorStateNoManipulatorState) {
  // GIVEN an empty RobotState
  ::bosdyn::api::RobotState robot_state;

  // WHEN we call getEndEffectorForce()
  const auto out = getManipulatorState(robot_state);
  // THEN the conversion does not succeed
  EXPECT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetManipulatorStateNoForceInHand) {
  // GIVEN an empty RobotState
  ::bosdyn::api::RobotState robot_state;
  robot_state.mutable_manipulator_state()->set_is_gripper_holding_item(true);
  robot_state.mutable_manipulator_state()->clear_estimated_end_effector_force_in_hand();

  // WHEN we call getEndEffectorForce()
  const auto out = getManipulatorState(robot_state);
  // THEN the conversion does not succeed
  EXPECT_THAT(out.has_value(), testing::IsTrue());

  EXPECT_THAT(out->estimated_end_effector_force_in_hand_is_set, testing::IsFalse());
}

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
}  // namespace spot_ros2::conversions::test
