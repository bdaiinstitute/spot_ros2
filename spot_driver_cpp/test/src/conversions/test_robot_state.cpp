// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver_cpp/conversions/robot_state.hpp>
#include <bosdyn/api/RobotState.pb.h>

namespace {
  ::bosdyn::api::BatteryState createBatteryState(const std::string& id, uint32_t percentage, uint32_t current, 
                                                 uint32_t voltage, double temp, ::bosdyn::api::BatteryState_Status status){
    ::bosdyn::api::BatteryState out;

    out.mutable_identifier() = id;
    out.mutable_charge_percentage() = percentage;
    out.mutable_current() = current;
    out.mutable_voltage() = voltage;
    out.add_temperatures().push_back(temp);
    out.status = status;

    return out;
  }

  ::bosdyn::api::WifiState createWifiState(::bosdyn::api::WiFiState_Mode mode, const std::string& essid){
    ::bosdyn::api::WifiState out;

    out.set_current_mode(); 
    out.mutable_essid() = essid;

    return out;
  }

    ::bosdyn::api::WifiState createWifiState(::bosdyn::api::WiFiState_Mode mode, const std::string& essid){
    ::bosdyn::api::WifiState out;

    out.set_current_mode(); 
    out.mutable_essid() = essid;

    return out;
  }

}

namespace spot_ros2::conversions::test {

TEST(RobotStateConversions, TestGetBatteryStates){
  // GIVEN a clock skew and a Robot State containing a BatteryState, a WifiState, FootState, and an EstopState
  const auto battery_state = createBatteryState("test_battery", 50, 10, 12, 80.0, ::bosdyn::api::BatteryState_Status::STATUS_DISCHARGING);
  // WHEN retrieving required fields from Robot State

  // THEN the returned ROS messages to be valid.

}

TEST(RobotStateConversions, TestGetJointStates_has_kinematic_state){}
TEST(RobotStateConversions, TestGetJointStates_no_kinematic_state){}

TEST(RobotStateConversions, TestGetTf_has_kinematic_state){}
TEST(RobotStateConversions, TestGetTf_no_kinematic_state){}

TEST(RobotStateConversions, TestGetOdomTwist_has_kinematic_state){}
TEST(RobotStateConversions, TestGetOdomTwist_no_kinematic_state){}

TEST(RobotStateConversions, TestGetOdom_has_kinematic_state){}
TEST(RobotStateConversions, TestGetOdom_no_kinematic_state){}

TEST(RobotStateConversions, TestGetPowerState_has_kinematic_state){}
TEST(RobotStateConversions, TestGetPowerState_no_kinematic_state){}

TEST(RobotStateConversions, TestGetSystemFaultState_has_kinematic_state){}
TEST(RobotStateConversions, TestGetSystemFaultState_no_kinematic_state){}

TEST(RobotStateConversions, TestGetManipulatorState_has_kinematic_state){}
TEST(RobotStateConversions, TestGetManipulatorState_no_kinematic_state){}

TEST(RobotStateConversions, TestGetEndEffectorForce_has_kinematic_state){}
TEST(RobotStateConversions, TestGetEndEffectorForce_no_kinematic_state){}

TEST(RobotStateConversions, TestGetBehaviorFaultState_has_kinematic_state){}
TEST(RobotStateConversions, TestGetBehaviorFaultState_no_kinematic_state){}
}
