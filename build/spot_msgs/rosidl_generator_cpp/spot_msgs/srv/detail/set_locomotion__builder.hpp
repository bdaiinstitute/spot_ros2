// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/SetLocomotion.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__SET_LOCOMOTION__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__SET_LOCOMOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/set_locomotion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLocomotion_Request_locomotion_mode
{
public:
  Init_SetLocomotion_Request_locomotion_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::SetLocomotion_Request locomotion_mode(::spot_msgs::srv::SetLocomotion_Request::_locomotion_mode_type arg)
  {
    msg_.locomotion_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::SetLocomotion_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::SetLocomotion_Request>()
{
  return spot_msgs::srv::builder::Init_SetLocomotion_Request_locomotion_mode();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLocomotion_Response_message
{
public:
  explicit Init_SetLocomotion_Response_message(::spot_msgs::srv::SetLocomotion_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::SetLocomotion_Response message(::spot_msgs::srv::SetLocomotion_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::SetLocomotion_Response msg_;
};

class Init_SetLocomotion_Response_success
{
public:
  Init_SetLocomotion_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLocomotion_Response_message success(::spot_msgs::srv::SetLocomotion_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetLocomotion_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::SetLocomotion_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::SetLocomotion_Response>()
{
  return spot_msgs::srv::builder::Init_SetLocomotion_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__SET_LOCOMOTION__BUILDER_HPP_
