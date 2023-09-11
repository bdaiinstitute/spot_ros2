// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/ListSounds.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__LIST_SOUNDS__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__LIST_SOUNDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/list_sounds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListSounds_Request>()
{
  return ::spot_msgs::srv::ListSounds_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_ListSounds_Response_message
{
public:
  explicit Init_ListSounds_Response_message(::spot_msgs::srv::ListSounds_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::ListSounds_Response message(::spot_msgs::srv::ListSounds_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::ListSounds_Response msg_;
};

class Init_ListSounds_Response_success
{
public:
  explicit Init_ListSounds_Response_success(::spot_msgs::srv::ListSounds_Response & msg)
  : msg_(msg)
  {}
  Init_ListSounds_Response_message success(::spot_msgs::srv::ListSounds_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ListSounds_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::ListSounds_Response msg_;
};

class Init_ListSounds_Response_names
{
public:
  Init_ListSounds_Response_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ListSounds_Response_success names(::spot_msgs::srv::ListSounds_Response::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_ListSounds_Response_success(msg_);
  }

private:
  ::spot_msgs::srv::ListSounds_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::ListSounds_Response>()
{
  return spot_msgs::srv::builder::Init_ListSounds_Response_names();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__LIST_SOUNDS__BUILDER_HPP_
