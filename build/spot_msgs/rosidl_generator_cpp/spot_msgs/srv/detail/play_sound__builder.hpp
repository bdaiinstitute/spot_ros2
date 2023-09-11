// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/PlaySound.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__PLAY_SOUND__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__PLAY_SOUND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/play_sound__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_PlaySound_Request_volume_multiplier
{
public:
  explicit Init_PlaySound_Request_volume_multiplier(::spot_msgs::srv::PlaySound_Request & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::PlaySound_Request volume_multiplier(::spot_msgs::srv::PlaySound_Request::_volume_multiplier_type arg)
  {
    msg_.volume_multiplier = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::PlaySound_Request msg_;
};

class Init_PlaySound_Request_name
{
public:
  Init_PlaySound_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaySound_Request_volume_multiplier name(::spot_msgs::srv::PlaySound_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_PlaySound_Request_volume_multiplier(msg_);
  }

private:
  ::spot_msgs::srv::PlaySound_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::PlaySound_Request>()
{
  return spot_msgs::srv::builder::Init_PlaySound_Request_name();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_PlaySound_Response_message
{
public:
  explicit Init_PlaySound_Response_message(::spot_msgs::srv::PlaySound_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::PlaySound_Response message(::spot_msgs::srv::PlaySound_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::PlaySound_Response msg_;
};

class Init_PlaySound_Response_success
{
public:
  Init_PlaySound_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaySound_Response_message success(::spot_msgs::srv::PlaySound_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlaySound_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::PlaySound_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::PlaySound_Response>()
{
  return spot_msgs::srv::builder::Init_PlaySound_Response_success();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__PLAY_SOUND__BUILDER_HPP_
