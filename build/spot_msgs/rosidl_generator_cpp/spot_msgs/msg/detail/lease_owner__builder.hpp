// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/LeaseOwner.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/lease_owner__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_LeaseOwner_user_name
{
public:
  explicit Init_LeaseOwner_user_name(::spot_msgs::msg::LeaseOwner & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::LeaseOwner user_name(::spot_msgs::msg::LeaseOwner::_user_name_type arg)
  {
    msg_.user_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::LeaseOwner msg_;
};

class Init_LeaseOwner_client_name
{
public:
  Init_LeaseOwner_client_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LeaseOwner_user_name client_name(::spot_msgs::msg::LeaseOwner::_client_name_type arg)
  {
    msg_.client_name = std::move(arg);
    return Init_LeaseOwner_user_name(msg_);
  }

private:
  ::spot_msgs::msg::LeaseOwner msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::LeaseOwner>()
{
  return spot_msgs::msg::builder::Init_LeaseOwner_client_name();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__BUILDER_HPP_
