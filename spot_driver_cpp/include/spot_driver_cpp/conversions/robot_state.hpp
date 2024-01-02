// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn_msgs/msg/manipulator_state.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
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
#include <string_view>
#include <tf2_msgs/msg/tf_message.hpp>

namespace spot_ros2 {

/**
 * @brief Mapping of Spot joint names to descriptive joint names to be used
 */
inline std::map<const std::string, const std::string> kFriendlyJointNames = {
    {"fl.hx", "front_left_hip_x"},  {"fl.hy", "front_left_hip_y"},  {"fl.kn", "front_left_knee"},
    {"fr.hx", "front_right_hip_x"}, {"fr.hy", "front_right_hip_y"}, {"fr.kn", "front_right_knee"},
    {"hl.hx", "rear_left_hip_x"},   {"hl.hy", "rear_left_hip_y"},   {"hl.kn", "rear_left_knee"},
    {"hr.hx", "rear_right_hip_x"},  {"hr.hy", "rear_right_hip_y"},  {"hr.kn", "rear_right_knee"},
    {"arm0.sh0", "arm_sh0"},        {"arm0.sh1", "arm_sh1"},        {"arm0.hr0", "arm_hr0"},
    {"arm0.el0", "arm_el0"},        {"arm0.el1", "arm_el1"},        {"arm0.wr0", "arm_wr0"},
    {"arm0.wr1", "arm_wr1"},        {"arm0.f1x", "arm_f1x"},
};

spot_msgs::msg::BatteryStateArray GetBatteryStates(const ::bosdyn::api::RobotState& robot_state,
                                                   const google::protobuf::Duration& clock_skew);

spot_msgs::msg::WiFiState GetWifiState(const ::bosdyn::api::RobotState& robot_state);

spot_msgs::msg::FootStateArray GetFootState(const ::bosdyn::api::RobotState& robot_state);

spot_msgs::msg::EStopStateArray GetEstopStates(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew);

std::optional<sensor_msgs::msg::JointState> GetJointStates(const ::bosdyn::api::RobotState& robot_state,
                                                           const google::protobuf::Duration& clock_skew,
                                                           const std::string& prefix);

std::optional<tf2_msgs::msg::TFMessage> GetTf(const ::bosdyn::api::RobotState& robot_state,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              const std::string& inverse_target_frame_id);

std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> GetOdomTwist(
    const ::bosdyn::api::RobotState& robot_state, const google::protobuf::Duration& clock_skew);

std::optional<nav_msgs::msg::Odometry> GetOdom(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                               bool is_using_vision);

std::optional<spot_msgs::msg::PowerState> GetPowerState(const ::bosdyn::api::RobotState& robot_state,
                                                        const google::protobuf::Duration& clock_skew);

std::optional<spot_msgs::msg::SystemFaultState> GetSystemFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                    const google::protobuf::Duration& clock_skew);

std::optional<bosdyn_msgs::msg::ManipulatorState> GetManipulatorState(const ::bosdyn::api::RobotState& robot_state);

std::optional<geometry_msgs::msg::Vector3Stamped> GetEndEffectorForce(const ::bosdyn::api::RobotState& robot_state,
                                                                      const google::protobuf::Duration& clock_skew,
                                                                      const std::string& prefix);

std::optional<spot_msgs::msg::BehaviorFaultState> GetBehaviorFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                        const google::protobuf::Duration& clock_skew);

}  // namespace spot_ros2
