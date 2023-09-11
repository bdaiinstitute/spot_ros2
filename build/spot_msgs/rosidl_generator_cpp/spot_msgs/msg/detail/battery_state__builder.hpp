// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/BatteryState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BATTERY_STATE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__BATTERY_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/battery_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryState_status
{
public:
  explicit Init_BatteryState_status(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::BatteryState status(::spot_msgs::msg::BatteryState::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_temperatures
{
public:
  explicit Init_BatteryState_temperatures(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_status temperatures(::spot_msgs::msg::BatteryState::_temperatures_type arg)
  {
    msg_.temperatures = std::move(arg);
    return Init_BatteryState_status(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_voltage
{
public:
  explicit Init_BatteryState_voltage(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_temperatures voltage(::spot_msgs::msg::BatteryState::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BatteryState_temperatures(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_current
{
public:
  explicit Init_BatteryState_current(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_voltage current(::spot_msgs::msg::BatteryState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BatteryState_voltage(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_estimated_runtime
{
public:
  explicit Init_BatteryState_estimated_runtime(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_current estimated_runtime(::spot_msgs::msg::BatteryState::_estimated_runtime_type arg)
  {
    msg_.estimated_runtime = std::move(arg);
    return Init_BatteryState_current(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_charge_percentage
{
public:
  explicit Init_BatteryState_charge_percentage(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_estimated_runtime charge_percentage(::spot_msgs::msg::BatteryState::_charge_percentage_type arg)
  {
    msg_.charge_percentage = std::move(arg);
    return Init_BatteryState_estimated_runtime(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_identifier
{
public:
  explicit Init_BatteryState_identifier(::spot_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_charge_percentage identifier(::spot_msgs::msg::BatteryState::_identifier_type arg)
  {
    msg_.identifier = std::move(arg);
    return Init_BatteryState_charge_percentage(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_header
{
public:
  Init_BatteryState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryState_identifier header(::spot_msgs::msg::BatteryState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BatteryState_identifier(msg_);
  }

private:
  ::spot_msgs::msg::BatteryState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::BatteryState>()
{
  return spot_msgs::msg::builder::Init_BatteryState_header();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__BATTERY_STATE__BUILDER_HPP_
