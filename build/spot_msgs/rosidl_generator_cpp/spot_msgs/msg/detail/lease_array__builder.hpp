// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/LeaseArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_ARRAY__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/lease_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_LeaseArray_resources
{
public:
  Init_LeaseArray_resources()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::msg::LeaseArray resources(::spot_msgs::msg::LeaseArray::_resources_type arg)
  {
    msg_.resources = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::LeaseArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::LeaseArray>()
{
  return spot_msgs::msg::builder::Init_LeaseArray_resources();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_ARRAY__BUILDER_HPP_
