// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/ClearBehaviorFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__CLEAR_BEHAVIOR_FAULT__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__CLEAR_BEHAVIOR_FAULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/clear_behavior_fault__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ClearBehaviorFault_Request_id
{
public:
  Init_ClearBehaviorFault_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::ClearBehaviorFault_Request id(::spot_msgs::srv::ClearBehaviorFault_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ClearBehaviorFault_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ClearBehaviorFault_Request>()
{
  return spot_msgs::srv::builder::Init_ClearBehaviorFault_Request_id();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ClearBehaviorFault_Response_message
{
public:
  explicit Init_ClearBehaviorFault_Response_message(::spot_msgs::srv::ClearBehaviorFault_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::ClearBehaviorFault_Response message(::spot_msgs::srv::ClearBehaviorFault_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ClearBehaviorFault_Response msg_;
};

class Init_ClearBehaviorFault_Response_success
{
public:
  Init_ClearBehaviorFault_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ClearBehaviorFault_Response_message success(::spot_msgs::srv::ClearBehaviorFault_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ClearBehaviorFault_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::ClearBehaviorFault_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ClearBehaviorFault_Response>()
{
  return spot_msgs::srv::builder::Init_ClearBehaviorFault_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__CLEAR_BEHAVIOR_FAULT__BUILDER_HPP_
