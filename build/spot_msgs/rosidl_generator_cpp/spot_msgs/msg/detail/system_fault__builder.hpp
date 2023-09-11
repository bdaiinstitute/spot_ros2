// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/SystemFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/system_fault__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_SystemFault_severity
{
public:
  explicit Init_SystemFault_severity(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::SystemFault severity(::spot_msgs::msg::SystemFault::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_attributes
{
public:
  explicit Init_SystemFault_attributes(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  Init_SystemFault_severity attributes(::spot_msgs::msg::SystemFault::_attributes_type arg)
  {
    msg_.attributes = std::move(arg);
    return Init_SystemFault_severity(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_error_message
{
public:
  explicit Init_SystemFault_error_message(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  Init_SystemFault_attributes error_message(::spot_msgs::msg::SystemFault::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_SystemFault_attributes(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_uid
{
public:
  explicit Init_SystemFault_uid(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  Init_SystemFault_error_message uid(::spot_msgs::msg::SystemFault::_uid_type arg)
  {
    msg_.uid = std::move(arg);
    return Init_SystemFault_error_message(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_code
{
public:
  explicit Init_SystemFault_code(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  Init_SystemFault_uid code(::spot_msgs::msg::SystemFault::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_SystemFault_uid(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_duration
{
public:
  explicit Init_SystemFault_duration(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  Init_SystemFault_code duration(::spot_msgs::msg::SystemFault::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_SystemFault_code(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_name
{
public:
  explicit Init_SystemFault_name(::spot_msgs::msg::SystemFault & msg)
  : msg_(msg)
  {}
  Init_SystemFault_duration name(::spot_msgs::msg::SystemFault::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_SystemFault_duration(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

class Init_SystemFault_header
{
public:
  Init_SystemFault_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemFault_name header(::spot_msgs::msg::SystemFault::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SystemFault_name(msg_);
  }

private:
  ::spot_msgs::msg::SystemFault msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::SystemFault>()
{
  return spot_msgs::msg::builder::Init_SystemFault_header();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__BUILDER_HPP_
