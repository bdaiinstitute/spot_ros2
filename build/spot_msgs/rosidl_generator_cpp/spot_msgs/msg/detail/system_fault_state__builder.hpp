// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/SystemFaultState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT_STATE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/system_fault_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_SystemFaultState_historical_faults
{
public:
  explicit Init_SystemFaultState_historical_faults(::spot_msgs::msg::SystemFaultState & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::SystemFaultState historical_faults(::spot_msgs::msg::SystemFaultState::_historical_faults_type arg)
  {
    msg_.historical_faults = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::SystemFaultState msg_;
};

class Init_SystemFaultState_faults
{
public:
  Init_SystemFaultState_faults()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemFaultState_historical_faults faults(::spot_msgs::msg::SystemFaultState::_faults_type arg)
  {
    msg_.faults = std::move(arg);
    return Init_SystemFaultState_historical_faults(msg_);
  }

private:
  ::spot_msgs::msg::SystemFaultState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::SystemFaultState>()
{
  return spot_msgs::msg::builder::Init_SystemFaultState_faults();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT_STATE__BUILDER_HPP_
