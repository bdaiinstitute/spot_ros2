// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/BehaviorFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/behavior_fault__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_BehaviorFault_status
{
public:
  explicit Init_BehaviorFault_status(::spot_msgs::msg::BehaviorFault & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::BehaviorFault status(::spot_msgs::msg::BehaviorFault::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::BehaviorFault msg_;
};

class Init_BehaviorFault_cause
{
public:
  explicit Init_BehaviorFault_cause(::spot_msgs::msg::BehaviorFault & msg)
  : msg_(msg)
  {}
  Init_BehaviorFault_status cause(::spot_msgs::msg::BehaviorFault::_cause_type arg)
  {
    msg_.cause = std::move(arg);
    return Init_BehaviorFault_status(msg_);
  }

private:
  ::spot_msgs::msg::BehaviorFault msg_;
};

class Init_BehaviorFault_behavior_fault_id
{
public:
  explicit Init_BehaviorFault_behavior_fault_id(::spot_msgs::msg::BehaviorFault & msg)
  : msg_(msg)
  {}
  Init_BehaviorFault_cause behavior_fault_id(::spot_msgs::msg::BehaviorFault::_behavior_fault_id_type arg)
  {
    msg_.behavior_fault_id = std::move(arg);
    return Init_BehaviorFault_cause(msg_);
  }

private:
  ::spot_msgs::msg::BehaviorFault msg_;
};

class Init_BehaviorFault_header
{
public:
  Init_BehaviorFault_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BehaviorFault_behavior_fault_id header(::spot_msgs::msg::BehaviorFault::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BehaviorFault_behavior_fault_id(msg_);
  }

private:
  ::spot_msgs::msg::BehaviorFault msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::BehaviorFault>()
{
  return spot_msgs::msg::builder::Init_BehaviorFault_header();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__BUILDER_HPP_
