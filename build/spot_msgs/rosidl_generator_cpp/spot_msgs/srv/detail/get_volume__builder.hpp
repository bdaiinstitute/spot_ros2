// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/GetVolume.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GET_VOLUME__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__GET_VOLUME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/get_volume__struct.hpp"
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
auto build<::spot_msgs::srv::GetVolume_Request>()
{
  return ::spot_msgs::srv::GetVolume_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetVolume_Response_message
{
public:
  explicit Init_GetVolume_Response_message(::spot_msgs::srv::GetVolume_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::GetVolume_Response message(::spot_msgs::srv::GetVolume_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GetVolume_Response msg_;
};

class Init_GetVolume_Response_success
{
public:
  explicit Init_GetVolume_Response_success(::spot_msgs::srv::GetVolume_Response & msg)
  : msg_(msg)
  {}
  Init_GetVolume_Response_message success(::spot_msgs::srv::GetVolume_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetVolume_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::GetVolume_Response msg_;
};

class Init_GetVolume_Response_volume
{
public:
  Init_GetVolume_Response_volume()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetVolume_Response_success volume(::spot_msgs::srv::GetVolume_Response::_volume_type arg)
  {
    msg_.volume = std::move(arg);
    return Init_GetVolume_Response_success(msg_);
  }

private:
  ::spot_msgs::srv::GetVolume_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GetVolume_Response>()
{
  return spot_msgs::srv::builder::Init_GetVolume_Response_volume();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__GET_VOLUME__BUILDER_HPP_
