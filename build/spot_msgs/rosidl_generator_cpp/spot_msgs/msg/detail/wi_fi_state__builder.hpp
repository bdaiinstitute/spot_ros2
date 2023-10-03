// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/WiFiState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__WI_FI_STATE__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__WI_FI_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/wi_fi_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_WiFiState_essid
{
public:
  explicit Init_WiFiState_essid(::spot_msgs::msg::WiFiState & msg)
  : msg_(msg)
  {}
  ::spot_msgs::msg::WiFiState essid(::spot_msgs::msg::WiFiState::_essid_type arg)
  {
    msg_.essid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::WiFiState msg_;
};

class Init_WiFiState_current_mode
{
public:
  Init_WiFiState_current_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WiFiState_essid current_mode(::spot_msgs::msg::WiFiState::_current_mode_type arg)
  {
    msg_.current_mode = std::move(arg);
    return Init_WiFiState_essid(msg_);
  }

private:
  ::spot_msgs::msg::WiFiState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::WiFiState>()
{
  return spot_msgs::msg::builder::Init_WiFiState_current_mode();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__WI_FI_STATE__BUILDER_HPP_
