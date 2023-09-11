// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/PowerState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__POWER_STATE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__POWER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/power_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_PowerState_locomotion_estimated_runtime
{
public:
  explicit Init_PowerState_locomotion_estimated_runtime(::spot_msgs::msg::PowerState & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::PowerState locomotion_estimated_runtime(::spot_msgs::msg::PowerState::_locomotion_estimated_runtime_type arg)
  {
    msg_.locomotion_estimated_runtime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::PowerState msg_;
};

class Init_PowerState_locomotion_charge_percentage
{
public:
  explicit Init_PowerState_locomotion_charge_percentage(::spot_msgs::msg::PowerState & msg)
  : msg_(msg)
  {}
  Init_PowerState_locomotion_estimated_runtime locomotion_charge_percentage(::spot_msgs::msg::PowerState::_locomotion_charge_percentage_type arg)
  {
    msg_.locomotion_charge_percentage = std::move(arg);
    return Init_PowerState_locomotion_estimated_runtime(msg_);
  }

private:
  ::spot_msgs::msg::PowerState msg_;
};

class Init_PowerState_shore_power_state
{
public:
  explicit Init_PowerState_shore_power_state(::spot_msgs::msg::PowerState & msg)
  : msg_(msg)
  {}
  Init_PowerState_locomotion_charge_percentage shore_power_state(::spot_msgs::msg::PowerState::_shore_power_state_type arg)
  {
    msg_.shore_power_state = std::move(arg);
    return Init_PowerState_locomotion_charge_percentage(msg_);
  }

private:
  ::spot_msgs::msg::PowerState msg_;
};

class Init_PowerState_motor_power_state
{
public:
  explicit Init_PowerState_motor_power_state(::spot_msgs::msg::PowerState & msg)
  : msg_(msg)
  {}
  Init_PowerState_shore_power_state motor_power_state(::spot_msgs::msg::PowerState::_motor_power_state_type arg)
  {
    msg_.motor_power_state = std::move(arg);
    return Init_PowerState_shore_power_state(msg_);
  }

private:
  ::spot_msgs::msg::PowerState msg_;
};

class Init_PowerState_header
{
public:
  Init_PowerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PowerState_motor_power_state header(::spot_msgs::msg::PowerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PowerState_motor_power_state(msg_);
  }

private:
  ::spot_msgs::msg::PowerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::PowerState>()
{
  return spot_msgs::msg::builder::Init_PowerState_header();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__POWER_STATE__BUILDER_HPP_
