// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/Lease.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/lease__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_Lease_sequence
{
public:
  explicit Init_Lease_sequence(::spot_msgs::msg::Lease & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::Lease sequence(::spot_msgs::msg::Lease::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::Lease msg_;
};

class Init_Lease_epoch
{
public:
  explicit Init_Lease_epoch(::spot_msgs::msg::Lease & msg)
  : msg_(msg)
  {}
  Init_Lease_sequence epoch(::spot_msgs::msg::Lease::_epoch_type arg)
  {
    msg_.epoch = std::move(arg);
    return Init_Lease_sequence(msg_);
  }

private:
  ::spot_msgs::msg::Lease msg_;
};

class Init_Lease_resource
{
public:
  Init_Lease_resource()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Lease_epoch resource(::spot_msgs::msg::Lease::_resource_type arg)
  {
    msg_.resource = std::move(arg);
    return Init_Lease_epoch(msg_);
  }

private:
  ::spot_msgs::msg::Lease msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::Lease>()
{
  return spot_msgs::msg::builder::Init_Lease_resource();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE__BUILDER_HPP_
