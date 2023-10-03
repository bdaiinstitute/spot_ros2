// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/FootStateArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FOOT_STATE_ARRAY__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__FOOT_STATE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/foot_state_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_FootStateArray_states
{
public:
  Init_FootStateArray_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::msg::FootStateArray states(::spot_msgs::msg::FootStateArray::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::FootStateArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::FootStateArray>()
{
  return spot_msgs::msg::builder::Init_FootStateArray_states();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__FOOT_STATE_ARRAY__BUILDER_HPP_
