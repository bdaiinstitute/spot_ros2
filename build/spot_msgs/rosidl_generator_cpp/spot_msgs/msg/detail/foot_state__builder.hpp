// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/foot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_FootState_contact
{
public:
  explicit Init_FootState_contact(::spot_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::FootState contact(::spot_msgs::msg::FootState::_contact_type arg)
  {
    msg_.contact = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::FootState msg_;
};

class Init_FootState_foot_position_rt_body
{
public:
  Init_FootState_foot_position_rt_body()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FootState_contact foot_position_rt_body(::spot_msgs::msg::FootState::_foot_position_rt_body_type arg)
  {
    msg_.foot_position_rt_body = std::move(arg);
    return Init_FootState_contact(msg_);
  }

private:
  ::spot_msgs::msg::FootState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::FootState>()
{
  return spot_msgs::msg::builder::Init_FootState_foot_position_rt_body();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_
