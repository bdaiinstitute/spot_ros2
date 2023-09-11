// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from spot_msgs:msg/BatteryStateArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BATTERY_STATE_ARRAY__BUILDER_HPP_
#define SPOT_MSGS__MSG__DETAIL__BATTERY_STATE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "spot_msgs/msg/detail/battery_state_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace spot_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryStateArray_battery_states
{
public:
  Init_BatteryStateArray_battery_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::spot_msgs::msg::BatteryStateArray battery_states(::spot_msgs::msg::BatteryStateArray::_battery_states_type arg)
  {
    msg_.battery_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::spot_msgs::msg::BatteryStateArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::spot_msgs::msg::BatteryStateArray>()
{
  return spot_msgs::msg::builder::Init_BatteryStateArray_battery_states();
}

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__BATTERY_STATE_ARRAY__BUILDER_HPP_
