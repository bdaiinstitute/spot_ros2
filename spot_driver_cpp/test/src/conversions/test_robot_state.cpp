// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/map.h>
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

void setFootState(::bosdyn::api::FootState* foot_state, const double x, const double y, const double z,
                  const ::bosdyn::api::FootState_Contact contact) {
  foot_state->mutable_foot_position_rt_body()->set_x(x);
  foot_state->mutable_foot_position_rt_body()->set_y(y);
  foot_state->mutable_foot_position_rt_body()->set_z(z);
  foot_state->set_contact(contact);
}

void setJointState(::bosdyn::api::JointState* joint_state, const std::string& name, const double position,
                   const double velocity, const double acceleration, const double load) {
  *joint_state->mutable_name() = name;
  joint_state->mutable_position()->set_value(position);
  joint_state->mutable_velocity()->set_value(velocity);
  joint_state->mutable_acceleration()->set_value(acceleration);
  joint_state->mutable_load()->set_value(load);
}

void addTransform(::bosdyn::api::FrameTreeSnapshot* mutable_frame_tree_snapshot, const std::string& child_name,
                  const std::string& parent_name, const double x, const double y, const double z, const double qw,
                  const double qx, const double qy, const double qz) {
  auto edge_map = mutable_frame_tree_snapshot->mutable_child_to_parent_edge_map();
  ::bosdyn::api::FrameTreeSnapshot_ParentEdge edge;
  *edge.mutable_parent_frame_name() = parent_name;
  auto pos = edge.mutable_parent_tform_child()->mutable_position();
  pos->set_x(x);
  pos->set_y(y);
  pos->set_z(z);
  auto rot = edge.mutable_parent_tform_child()->mutable_rotation();
  rot->set_w(qw);
  rot->set_x(qx);
  rot->set_y(qy);
  rot->set_z(qz);
  // (*edge_map)[child_name] = std::move(edge);
  edge_map->insert(google::protobuf::MapPair{child_name, edge});
}

void addRootFrame(::bosdyn::api::FrameTreeSnapshot* mutable_frame_tree_snapshot, const std::string& root_frame) {
  auto edge_map = mutable_frame_tree_snapshot->mutable_child_to_parent_edge_map();
  ::bosdyn::api::FrameTreeSnapshot_ParentEdge root_edge;
  edge_map->insert(google::protobuf::MapPair{root_frame, root_edge});
}

void addBodyVelocityOdom(::bosdyn::api::KinematicState* mutable_kinematic_state, double x, double y, double z,
                         double rx, double ry, double rz) {
  auto velocity_angular = mutable_kinematic_state->mutable_velocity_of_body_in_odom()->mutable_angular();
  velocity_angular->set_x(x);
  velocity_angular->set_y(y);
  velocity_angular->set_z(z);
  auto velocity_linear = mutable_kinematic_state->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(rx);
  velocity_linear->set_y(ry);
  velocity_linear->set_z(rz);
}

void addAcquisitionTimestamp(::bosdyn::api::KinematicState* mutable_kinematic_state, int64_t seconds, int nanoseconds) {
  mutable_kinematic_state->mutable_acquisition_timestamp()->set_seconds(seconds);
  mutable_kinematic_state->mutable_acquisition_timestamp()->set_nanos(nanoseconds);
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
  EXPECT_THAT(first_battery_state.charge_percentage, testing::DoubleEq(50.0));
  EXPECT_THAT(first_battery_state.current, testing::DoubleEq(10.0));
  EXPECT_THAT(first_battery_state.voltage, testing::DoubleEq(12.0));
  ASSERT_THAT(first_battery_state.temperatures, testing::SizeIs(1ul));
  EXPECT_THAT(first_battery_state.temperatures.at(0), testing::DoubleEq(80.0));
}

TEST(RobotStateConversions, TestGetWifiState) {
  // GIVEN a RobotState which contains a valid CommsState field
  constexpr auto input_mode = ::bosdyn::api::WiFiState_Mode::WiFiState_Mode_MODE_CLIENT;
  constexpr auto input_essid = "some_value";
  ::bosdyn::api::RobotState robot_state;
  auto comms_state = robot_state.add_comms_states();
  comms_state->mutable_wifi_state()->CopyFrom(createWifiState(input_mode, input_essid));

  // WHEN we create a WiFiState ROS message from the RobotState
  const spot_msgs::msg::WiFiState out = getWifiState(robot_state);

  // THEN the ROS message contains the same values for the ESSID and mode fields as were set in the input RobotState
  EXPECT_THAT(out.current_mode, testing::Eq(input_mode));
  EXPECT_THAT(out.essid, testing::StrEq(input_essid));
}

TEST(RobotStateConversions, TestGetFootState) {
  // GIVEN a RobotState that contains four unique foot states
  ::bosdyn::api::RobotState robot_state;
  setFootState(robot_state.add_foot_state(), 1.0, 2.0, 3.0, ::bosdyn::api::FootState::CONTACT_MADE);
  setFootState(robot_state.add_foot_state(), 4.0, 5.0, 6.0, ::bosdyn::api::FootState::CONTACT_LOST);
  setFootState(robot_state.add_foot_state(), 7.0, 8.0, 9.0, ::bosdyn::api::FootState::CONTACT_MADE);
  setFootState(robot_state.add_foot_state(), 10.0, 11.0, 12.0, ::bosdyn::api::FootState::CONTACT_LOST);

  // WHEN we create a FootStateArray ROS message from the RobotState
  const auto out = getFootState(robot_state);

  // THEN the output message contains four foot states
  ASSERT_THAT(out.states, testing::SizeIs(4));

  // THEN each output foot state matches the corresponding input, and is listed in the same order
  const auto& first_foot_state = out.states.at(0);
  EXPECT_THAT(first_foot_state.contact, testing::Eq(spot_msgs::msg::FootState::CONTACT_MADE));
  EXPECT_THAT(first_foot_state.foot_position_rt_body.x, testing::DoubleEq(1.0));
  EXPECT_THAT(first_foot_state.foot_position_rt_body.y, testing::DoubleEq(2.0));
  EXPECT_THAT(first_foot_state.foot_position_rt_body.z, testing::DoubleEq(3.0));
  const auto& second_foot_state = out.states.at(1);
  EXPECT_THAT(second_foot_state.contact, testing::Eq(spot_msgs::msg::FootState::CONTACT_LOST));
  EXPECT_THAT(second_foot_state.foot_position_rt_body.x, testing::DoubleEq(4.0));
  EXPECT_THAT(second_foot_state.foot_position_rt_body.y, testing::DoubleEq(5.0));
  EXPECT_THAT(second_foot_state.foot_position_rt_body.z, testing::DoubleEq(6.0));
  const auto& third_foot_state = out.states.at(2);
  EXPECT_THAT(third_foot_state.contact, testing::Eq(spot_msgs::msg::FootState::CONTACT_MADE));
  EXPECT_THAT(third_foot_state.foot_position_rt_body.x, testing::DoubleEq(7.0));
  EXPECT_THAT(third_foot_state.foot_position_rt_body.y, testing::DoubleEq(8.0));
  EXPECT_THAT(third_foot_state.foot_position_rt_body.z, testing::DoubleEq(9.0));
  const auto& fourth_foot_state = out.states.at(3);
  EXPECT_THAT(fourth_foot_state.contact, testing::Eq(spot_msgs::msg::FootState::CONTACT_LOST));
  EXPECT_THAT(fourth_foot_state.foot_position_rt_body.x, testing::DoubleEq(10.0));
  EXPECT_THAT(fourth_foot_state.foot_position_rt_body.y, testing::DoubleEq(11.0));
  EXPECT_THAT(fourth_foot_state.foot_position_rt_body.z, testing::DoubleEq(12.0));
}

TEST(RobotStateConversions, TestGetFootStateNoFootStates) {
  // GIVEN a RobotState that does not contain foot states
  ::bosdyn::api::RobotState robot_state;

  // WHEN we create a FootStateArray ROS message from the RobotState
  const auto out = getFootState(robot_state);

  // THEN the output message contains an empty array
  ASSERT_THAT(out.states, testing::IsEmpty());
}

TEST(RobotStateConversions, TestGetEStopStates) {
  // GIVEN a RobotState that contains two different EStopStates
  ::bosdyn::api::RobotState robot_state;

  ::bosdyn::api::EStopState estop_state_first;
  estop_state_first.mutable_timestamp()->set_seconds(13);
  estop_state_first.mutable_timestamp()->set_nanos(0);
  estop_state_first.set_name("estop_name");
  estop_state_first.set_type(::bosdyn::api::EStopState_Type::EStopState_Type_TYPE_HARDWARE);
  estop_state_first.set_state(::bosdyn::api::EStopState_State::EStopState_State_STATE_ESTOPPED);
  estop_state_first.set_state_description("some_text");
  robot_state.mutable_estop_states()->Add(std::move(estop_state_first));

  ::bosdyn::api::EStopState estop_state_second;
  estop_state_second.mutable_timestamp()->set_seconds(12);
  estop_state_second.mutable_timestamp()->set_nanos(0);
  estop_state_second.set_name("second_estop_name");
  estop_state_second.set_type(::bosdyn::api::EStopState_Type::EStopState_Type_TYPE_SOFTWARE);
  estop_state_second.set_state(::bosdyn::api::EStopState_State::EStopState_State_STATE_NOT_ESTOPPED);
  estop_state_second.set_state_description("other_text");
  robot_state.mutable_estop_states()->Add(std::move(estop_state_second));

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create an EStopStatesArray ROS message from the RobotState
  const auto out = getEstopStates(robot_state, clock_skew);

  // THEN the output message contains both estop states
  ASSERT_THAT(out.estop_states, testing::SizeIs(2));

  // THEN each output estop state contains equivalent data to its corresponding input
  // TODO(schornakj): validate timestamps too
  EXPECT_THAT(
      out.estop_states,
      testing::Contains(testing::AllOf(
          testing::Field("name", &spot_msgs::msg::EStopState::name, testing::StrEq("estop_name")),
          testing::Field("type", &spot_msgs::msg::EStopState::type, spot_msgs::msg::EStopState::TYPE_HARDWARE),
          testing::Field("state", &spot_msgs::msg::EStopState::state, spot_msgs::msg::EStopState::STATE_ESTOPPED),
          testing::Field("state_description", &spot_msgs::msg::EStopState::state_description,
                         testing::StrEq("some_text")))));
  EXPECT_THAT(
      out.estop_states,
      testing::Contains(testing::AllOf(
          testing::Field("name", &spot_msgs::msg::EStopState::name, testing::StrEq("second_estop_name")),
          testing::Field("type", &spot_msgs::msg::EStopState::type, spot_msgs::msg::EStopState::TYPE_SOFTWARE),
          testing::Field("state", &spot_msgs::msg::EStopState::state, spot_msgs::msg::EStopState::STATE_NOT_ESTOPPED),
          testing::Field("state_description", &spot_msgs::msg::EStopState::state_description,
                         testing::StrEq("other_text")))));
}

TEST(RobotStateConversions, TestGetJointStates) {
  // GIVEN a RobotState containing two unique joint states
  ::bosdyn::api::RobotState robot_state;
  auto timestamp = robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp();
  timestamp->set_seconds(15);
  timestamp->set_nanos(0);
  setJointState(robot_state.mutable_kinematic_state()->add_joint_states(), "fl.hx", 0.1, 0.2, 0.3, 0.4);
  setJointState(robot_state.mutable_kinematic_state()->add_joint_states(), "arm0.wr0", 0.5, 0.6, 0.7, 0.8);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a prefix
  constexpr auto prefix = "my_prefix/";

  // WHEN we create an JointState ROS message from the RobotState
  const auto out = getJointStates(robot_state, clock_skew, prefix);

  // THEN a message is created
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN each vector element of the joint state contains a number of elements matching the number of input joint states
  ASSERT_THAT(out->name, testing::SizeIs(2));
  ASSERT_THAT(out->position, testing::SizeIs(2));
  ASSERT_THAT(out->velocity, testing::SizeIs(2));
  ASSERT_THAT(out->effort, testing::SizeIs(2));

  // THEN the joint names were converted to the "friendly" names used in the URDF
  ASSERT_THAT(out->name, testing::UnorderedElementsAre("my_prefix/front_left_hip_x", "my_prefix/arm_wr0"));

  // THEN the position, velocity, and effort corresponding to each named joint match the input joint states
  const std::size_t index1 =
      std::distance(out->name.cbegin(), std::find(out->name.cbegin(), out->name.cend(), "my_prefix/front_left_hip_x"));
  EXPECT_THAT(out->position.at(index1), testing::DoubleEq(0.1));
  EXPECT_THAT(out->velocity.at(index1), testing::DoubleEq(0.2));
  EXPECT_THAT(out->effort.at(index1), testing::DoubleEq(0.4));

  const std::size_t index2 =
      std::distance(out->name.cbegin(), std::find(out->name.cbegin(), out->name.cend(), "my_prefix/arm_wr0"));
  EXPECT_THAT(out->position.at(index2), testing::DoubleEq(0.5));
  EXPECT_THAT(out->velocity.at(index2), testing::DoubleEq(0.6));
  EXPECT_THAT(out->effort.at(index2), testing::DoubleEq(0.8));
}

TEST(RobotStateConversions, TestGetJointStatesNoJointStates) {
  // GIVEN a RobotState that does not contain any joint states, but does contains some other kinematic state info
  ::bosdyn::api::RobotState robot_state;
  auto timestamp = robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp();
  timestamp->set_seconds(15);
  timestamp->set_nanos(0);

  // GIVEN some nominal clock skew and prefix
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  constexpr auto prefix = "prefix/";

  // WHEN we create a JointState ROS message from the RobotState
  const auto out = getJointStates(robot_state, clock_skew, prefix);

  // THEN the message is created and contains a timestamp, but does not contain any joint state data
  ASSERT_THAT(out.has_value(), testing::IsTrue());
  EXPECT_THAT(out->header.stamp.sec, testing::Eq(14));
  EXPECT_THAT(out->header.stamp.nanosec, testing::Eq(0U));
  EXPECT_THAT(out->name, testing::IsEmpty());
  EXPECT_THAT(out->position, testing::IsEmpty());
  EXPECT_THAT(out->velocity, testing::IsEmpty());
  EXPECT_THAT(out->effort, testing::IsEmpty());
}

TEST(RobotStateConversions, TestGetJointStatesNoKinematicState) {
  // GIVEN a RobotState that does not contain any kinematic state data whatsoever
  ::bosdyn::api::RobotState robot_state;

  // GIVEN some nominal clock skew and prefix
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  constexpr auto prefix = "prefix/";

  // WHEN we attempt to create a JointState ROS message from the RobotState
  const auto out = getJointStates(robot_state, clock_skew, prefix);

  // THEN no ROS message is output
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetTf) {}

TEST(RobotStateConversions, TestGetOdomTwist) {
  // GIVEN a RobotState that contains info about the velocity of the body in the odom frame
  ::bosdyn::api::RobotState robot_state;
  auto acquisition_timestamp = robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp();
  acquisition_timestamp->set_seconds(99);
  acquisition_timestamp->set_nanos(0);
  auto velocity_angular = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_angular();
  velocity_angular->set_x(1.0);
  velocity_angular->set_y(2.0);
  velocity_angular->set_z(3.0);
  auto velocity_linear = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(4.0);
  velocity_linear->set_y(5.0);
  velocity_linear->set_z(6.0);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TwistWithCovarianceStamped ROS message
  const auto out = getOdomTwist(robot_state, clock_skew);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN the output twist matches the velocity in the robot state
  EXPECT_THAT(out->twist.twist.angular.x, testing::DoubleEq(1.0));
  EXPECT_THAT(out->twist.twist.angular.y, testing::DoubleEq(2.0));
  EXPECT_THAT(out->twist.twist.angular.z, testing::DoubleEq(3.0));
  EXPECT_THAT(out->twist.twist.linear.x, testing::DoubleEq(4.0));
  EXPECT_THAT(out->twist.twist.linear.y, testing::DoubleEq(5.0));
  EXPECT_THAT(out->twist.twist.linear.z, testing::DoubleEq(6.0));
  EXPECT_THAT(out->header.stamp,
              testing::AllOf(testing::Field("sec", &builtin_interfaces::msg::Time::sec, testing::Eq(98)),
                             testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec, testing::Eq(0u))));
}

TEST(RobotStateConversions, TestGetOdomTwistNoBodyVelocityInRobotState) {
  // GIVEN a RobotState where there is some kinematic state info but no info about the velocity of the body in the odom
  // frame
  ::bosdyn::api::RobotState robot_state;
  auto acquisition_timestamp = robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp();
  acquisition_timestamp->set_seconds(99);
  acquisition_timestamp->set_nanos(0);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we attempt to create a TwistWithCovarianceStamped ROS message
  const auto out = getOdomTwist(robot_state, clock_skew);

  // THEN no message is output
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetOdomInOdomFrame) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState
  ::bosdyn::api::RobotState robot_state;
  // GIVEN a nonzero acquisition timestamp
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), 99, 0);
  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at a nonzero pose relative to the odom frame
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  // GIVEN the body is moving at a nonzero velocity relative to the odom frame
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  // GIVEN the frame tree snapshot is valid
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              testing::Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we create a nav_msgs::msg::Odometry ROS message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN the parent and child frame IDs of the Odometry pose refer to the correct frames and prepend the prefix to the
  // frame IDs
  EXPECT_THAT(out->header.frame_id, testing::StrEq("prefix/odom"));
  EXPECT_THAT(out->child_frame_id, testing::StrEq("prefix/body"));

  // THEN the Odometry ROS message contains the same pose and twist data as the RobotState
  EXPECT_THAT(out->pose.pose.position.x, testing::DoubleEq(1.0));
  EXPECT_THAT(out->pose.pose.position.y, testing::DoubleEq(2.0));
  EXPECT_THAT(out->pose.pose.position.z, testing::DoubleEq(3.0));
  EXPECT_THAT(out->pose.pose.orientation.w, testing::DoubleEq(1.0));
  EXPECT_THAT(out->pose.pose.orientation.x, testing::DoubleEq(0.0));
  EXPECT_THAT(out->pose.pose.orientation.y, testing::DoubleEq(0.0));
  EXPECT_THAT(out->pose.pose.orientation.z, testing::DoubleEq(0.0));
  EXPECT_THAT(out->twist.twist.angular.x, testing::DoubleEq(1.0));
  EXPECT_THAT(out->twist.twist.angular.y, testing::DoubleEq(2.0));
  EXPECT_THAT(out->twist.twist.angular.z, testing::DoubleEq(3.0));
  EXPECT_THAT(out->twist.twist.linear.x, testing::DoubleEq(4.0));
  EXPECT_THAT(out->twist.twist.linear.y, testing::DoubleEq(5.0));
  EXPECT_THAT(out->twist.twist.linear.z, testing::DoubleEq(6.0));
}

TEST(RobotStateConversions, TestGetOdomInVisionFrame) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that is fully populated with the required data about the robot's pose and velocity in the vision
  // frame
  ::bosdyn::api::RobotState robot_state;
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), 99, 0);
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "vision");
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "vision", 1.0, 2.0, 3.0,
               1.0, 0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              testing::Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we create a nav_msgs::msg::Odometry ROS message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", true);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), testing::IsTrue());

  // THEN the parent and child frame IDs of the Odometry pose refer to the correct frames and prepend the prefix to the
  // frame IDs
  EXPECT_THAT(out->header.frame_id, testing::StrEq("prefix/vision"));
  EXPECT_THAT(out->child_frame_id, testing::StrEq("prefix/body"));

  // THEN the Odometry ROS message contains the same pose and twist data as the RobotState
  EXPECT_THAT(out->pose.pose.position.x, testing::DoubleEq(1.0));
  EXPECT_THAT(out->pose.pose.position.y, testing::DoubleEq(2.0));
  EXPECT_THAT(out->pose.pose.position.z, testing::DoubleEq(3.0));
  EXPECT_THAT(out->pose.pose.orientation.w, testing::DoubleEq(1.0));
  EXPECT_THAT(out->pose.pose.orientation.x, testing::DoubleEq(0.0));
  EXPECT_THAT(out->pose.pose.orientation.y, testing::DoubleEq(0.0));
  EXPECT_THAT(out->pose.pose.orientation.z, testing::DoubleEq(0.0));
  EXPECT_THAT(out->twist.twist.angular.x, testing::DoubleEq(1.0));
  EXPECT_THAT(out->twist.twist.angular.y, testing::DoubleEq(2.0));
  EXPECT_THAT(out->twist.twist.angular.z, testing::DoubleEq(3.0));
  EXPECT_THAT(out->twist.twist.linear.x, testing::DoubleEq(4.0));
  EXPECT_THAT(out->twist.twist.linear.y, testing::DoubleEq(5.0));
  EXPECT_THAT(out->twist.twist.linear.z, testing::DoubleEq(6.0));
}

TEST(RobotStateConversions, TestGetOdomMissingAcquisitionTimestamp) {
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that contains some odom info, but does not contain an acquisition timestamp
  ::bosdyn::api::RobotState robot_state;
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              testing::Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetOdomMissingBodyVelocityOdom) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that does not contain info about the body velocity
  ::bosdyn::api::RobotState robot_state;
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), 99, 0);
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              testing::Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", true);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetOdomMissingTransforms) {
  // GIVEN a RobotState that does not contain a transform snapshot
  ::bosdyn::api::RobotState robot_state;
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), 99, 0);
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

TEST(RobotStateConversions, TestGetOdomInvalidTransformSnapshot) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that contains a transform snapshot which is invalid because it does not have a root frame
  ::bosdyn::api::RobotState robot_state;
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), 99, 0);
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "vision", 1.0, 2.0, 3.0,
               1.0, 0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              testing::Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::UNKNOWN_PARENT_FRAME_NAME));

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", true);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), testing::IsFalse());
}

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
                             testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(0u))));
  EXPECT_THAT(out->header.stamp,
              testing::AllOf(testing::Field("sec", &builtin_interfaces::msg::Time::sec, testing::Eq(59)),
                             testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec, testing::Eq(0u))));
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
                          testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec, testing::Eq(0u)))))),
              testing::Field(
                  "duration", &SystemFault::duration,
                  testing::AllOf(
                      testing::Field("sec", &builtin_interfaces::msg::Duration::sec, testing::Eq(15)),
                      testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(0u)))),
              testing::Field("name", &SystemFault::name, testing::StrEq("fault1")),
              testing::Field("code", &SystemFault::code, testing::Eq(19)),
              testing::Field("uid", &SystemFault::uid, testing::Eq(3ul)),
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
                          testing::Field("nanosec", &builtin_interfaces::msg::Time::nanosec, testing::Eq(0u)))))),
              testing::Field(
                  "duration", &SystemFault::duration,
                  testing::AllOf(
                      testing::Field("sec", &builtin_interfaces::msg::Duration::sec, testing::Eq(0)),
                      testing::Field("nanosec", &builtin_interfaces::msg::Duration::nanosec, testing::Eq(0u)))),
              testing::Field("name", &SystemFault::name, testing::StrEq("fault2")),
              testing::Field("code", &SystemFault::code, testing::Eq(55)),
              testing::Field("uid", &SystemFault::uid, testing::Eq(9ul)),
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
  EXPECT_THAT(out->gripper_open_percentage, testing::DoubleEq(50.0));
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
  EXPECT_THAT(out->vector.x, testing::DoubleEq(force.x()));
  EXPECT_THAT(out->vector.y, testing::DoubleEq(force.y()));
  EXPECT_THAT(out->vector.z, testing::DoubleEq(force.z()));
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
  EXPECT_THAT(first_fault.behavior_fault_id, testing::Eq(11u));
  EXPECT_THAT(first_fault.status,
              testing::Eq(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_CLEARABLE));
  EXPECT_THAT(first_fault.cause, testing::Eq(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_HARDWARE));
  EXPECT_THAT(first_fault.header.stamp.sec, testing::Eq(9));
  EXPECT_THAT(first_fault.header.stamp.nanosec, testing::Eq(0u));
  // THEN the second fault matches the second one added to the RobotState, and the clock skew is applied correctly
  const auto second_fault = out->faults.at(1);
  EXPECT_THAT(second_fault.behavior_fault_id, testing::Eq(12u));
  EXPECT_THAT(second_fault.status,
              testing::Eq(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_UNCLEARABLE));
  EXPECT_THAT(second_fault.cause, testing::Eq(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_FALL));
  EXPECT_THAT(second_fault.header.stamp.sec, testing::Eq(9));
  EXPECT_THAT(second_fault.header.stamp.nanosec, testing::Eq(0u));
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
