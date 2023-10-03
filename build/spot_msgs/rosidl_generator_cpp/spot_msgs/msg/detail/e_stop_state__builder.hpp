// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/EStopState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__E_STOP_STATE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__E_STOP_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/e_stop_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_EStopState_state_description
{
public:
  explicit Init_EStopState_state_description(::spot_msgs::msg::EStopState & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::EStopState state_description(::spot_msgs::msg::EStopState::_state_description_type arg)
  {
    msg_.state_description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::EStopState msg_;
};

class Init_EStopState_state
{
public:
  explicit Init_EStopState_state(::spot_msgs::msg::EStopState & msg)
  : msg_(msg)
  {}
  Init_EStopState_state_description state(::spot_msgs::msg::EStopState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_EStopState_state_description(msg_);
  }

private:
  ::spot_msgs::msg::EStopState msg_;
};

class Init_EStopState_type
{
public:
  explicit Init_EStopState_type(::spot_msgs::msg::EStopState & msg)
  : msg_(msg)
  {}
  Init_EStopState_state type(::spot_msgs::msg::EStopState::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_EStopState_state(msg_);
  }

private:
  ::spot_msgs::msg::EStopState msg_;
};

class Init_EStopState_name
{
public:
  explicit Init_EStopState_name(::spot_msgs::msg::EStopState & msg)
  : msg_(msg)
  {}
  Init_EStopState_type name(::spot_msgs::msg::EStopState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_EStopState_type(msg_);
  }

private:
  ::spot_msgs::msg::EStopState msg_;
};

class Init_EStopState_header
{
public:
  Init_EStopState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EStopState_name header(::spot_msgs::msg::EStopState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EStopState_name(msg_);
  }

private:
  ::spot_msgs::msg::EStopState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::EStopState>()
{
  return spot_msgs::msg::builder::Init_EStopState_header();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__E_STOP_STATE__BUILDER_HPP_
