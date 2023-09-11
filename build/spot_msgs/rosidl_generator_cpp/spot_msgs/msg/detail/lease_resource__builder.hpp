// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/LeaseResource.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/lease_resource__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_LeaseResource_lease_owner
{
public:
  explicit Init_LeaseResource_lease_owner(::spot_msgs::msg::LeaseResource & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::LeaseResource lease_owner(::spot_msgs::msg::LeaseResource::_lease_owner_type arg)
  {
    msg_.lease_owner = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::LeaseResource msg_;
};

class Init_LeaseResource_lease
{
public:
  explicit Init_LeaseResource_lease(::spot_msgs::msg::LeaseResource & msg)
  : msg_(msg)
  {}
  Init_LeaseResource_lease_owner lease(::spot_msgs::msg::LeaseResource::_lease_type arg)
  {
    msg_.lease = std::move(arg);
    return Init_LeaseResource_lease_owner(msg_);
  }

private:
  ::spot_msgs::msg::LeaseResource msg_;
};

class Init_LeaseResource_resource
{
public:
  Init_LeaseResource_resource()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LeaseResource_lease resource(::spot_msgs::msg::LeaseResource::_resource_type arg)
  {
    msg_.resource = std::move(arg);
    return Init_LeaseResource_lease(msg_);
  }

private:
  ::spot_msgs::msg::LeaseResource msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::LeaseResource>()
{
  return spot_msgs::msg::builder::Init_LeaseResource_resource();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__BUILDER_HPP_
