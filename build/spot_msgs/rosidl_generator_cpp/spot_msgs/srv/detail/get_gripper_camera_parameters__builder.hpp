// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:srv/GetGripperCameraParameters.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GET_GRIPPER_CAMERA_PARAMETERS__BUILDER_HPP_
#define SPOT_MSGS__SRV__DETAIL__GET_GRIPPER_CAMERA_PARAMETERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/srv/detail/get_gripper_camera_parameters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGripperCameraParameters_Request_request
{
public:
  Init_GetGripperCameraParameters_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::srv::GetGripperCameraParameters_Request request(::spot_msgs::srv::GetGripperCameraParameters_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GetGripperCameraParameters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GetGripperCameraParameters_Request>()
{
  return spot_msgs::srv::builder::Init_GetGripperCameraParameters_Request_request();
}

}  // namespace spot_msgs


namespace spot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGripperCameraParameters_Response_message
{
public:
  explicit Init_GetGripperCameraParameters_Response_message(::spot_msgs::srv::GetGripperCameraParameters_Response & msg)
  : msg_(msg)
  {}
  ::spot_msgs::srv::GetGripperCameraParameters_Response message(::spot_msgs::srv::GetGripperCameraParameters_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::srv::GetGripperCameraParameters_Response msg_;
};

class Init_GetGripperCameraParameters_Response_success
{
public:
  explicit Init_GetGripperCameraParameters_Response_success(::spot_msgs::srv::GetGripperCameraParameters_Response & msg)
  : msg_(msg)
  {}
  Init_GetGripperCameraParameters_Response_message success(::spot_msgs::srv::GetGripperCameraParameters_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetGripperCameraParameters_Response_message(msg_);
  }

private:
  ::spot_msgs::srv::GetGripperCameraParameters_Response msg_;
};

class Init_GetGripperCameraParameters_Response_response
{
public:
  Init_GetGripperCameraParameters_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGripperCameraParameters_Response_success response(::spot_msgs::srv::GetGripperCameraParameters_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_GetGripperCameraParameters_Response_success(msg_);
  }

private:
  ::spot_msgs::srv::GetGripperCameraParameters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::srv::GetGripperCameraParameters_Response>()
{
  return spot_msgs::srv::builder::Init_GetGripperCameraParameters_Response_response();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__GET_GRIPPER_CAMERA_PARAMETERS__BUILDER_HPP_
