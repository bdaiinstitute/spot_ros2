// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn_msgs/msg/manipulator_state.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include <string>
#include <string_view>
#include <tf2_msgs/msg/tf_message.hpp>

namespace spot_ros2 {

/**
 * @brief Mapping from the joint names used within the Spot API to the joint names used in the Spot driver and URDF.
 * @details As an example, "fl.hx" is the name of the joint in the Spot API, and "front_left_hip_x" is the name used for
 * this joint within the URDF.
 */
static const std::map<const std::string, const std::string> kFriendlyJointNames = {
    {"fl.hx", "front_left_hip_x"},  {"fl.hy", "front_left_hip_y"},  {"fl.kn", "front_left_knee"},
    {"fr.hx", "front_right_hip_x"}, {"fr.hy", "front_right_hip_y"}, {"fr.kn", "front_right_knee"},
    {"hl.hx", "rear_left_hip_x"},   {"hl.hy", "rear_left_hip_y"},   {"hl.kn", "rear_left_knee"},
    {"hr.hx", "rear_right_hip_x"},  {"hr.hy", "rear_right_hip_y"},  {"hr.kn", "rear_right_knee"},
    {"arm0.sh0", "arm_sh0"},        {"arm0.sh1", "arm_sh1"},        {"arm0.hr0", "arm_hr0"},
    {"arm0.el0", "arm_el0"},        {"arm0.el1", "arm_el1"},        {"arm0.wr0", "arm_wr0"},
    {"arm0.wr1", "arm_wr1"},        {"arm0.f1x", "arm_f1x"},
};

spot_msgs::msg::BatteryStateArray getBatteryStates(const ::bosdyn::api::RobotState& robot_state,
                                                   const google::protobuf::Duration& clock_skew);

spot_msgs::msg::WiFiState getWifiState(const ::bosdyn::api::RobotState& robot_state);

spot_msgs::msg::FootStateArray getFootState(const ::bosdyn::api::RobotState& robot_state);

spot_msgs::msg::EStopStateArray getEstopStates(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew);

std::optional<sensor_msgs::msg::JointState> getJointStates(const ::bosdyn::api::RobotState& robot_state,
                                                           const google::protobuf::Duration& clock_skew,
                                                           const std::string& prefix);

std::optional<tf2_msgs::msg::TFMessage> getTf(const ::bosdyn::api::RobotState& robot_state,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              const std::string& preferred_base_frame_id);

std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> getOdomTwist(
    const ::bosdyn::api::RobotState& robot_state, const google::protobuf::Duration& clock_skew);

std::optional<nav_msgs::msg::Odometry> getOdom(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                               bool is_using_vision);

std::optional<spot_msgs::msg::PowerState> getPowerState(const ::bosdyn::api::RobotState& robot_state,
                                                        const google::protobuf::Duration& clock_skew);

std::optional<spot_msgs::msg::SystemFaultState> getSystemFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                    const google::protobuf::Duration& clock_skew);

std::optional<bosdyn_msgs::msg::ManipulatorState> getManipulatorState(const ::bosdyn::api::RobotState& robot_state);

std::optional<geometry_msgs::msg::Vector3Stamped> getEndEffectorForce(const ::bosdyn::api::RobotState& robot_state,
                                                                      const google::protobuf::Duration& clock_skew,
                                                                      const std::string& prefix);

std::optional<spot_msgs::msg::BehaviorFaultState> getBehaviorFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                        const google::protobuf::Duration& clock_skew);

}  // namespace spot_ros2
