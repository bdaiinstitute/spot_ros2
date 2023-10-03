// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from spot_msgs:msg/EStopStateArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "spot_msgs/msg/detail/e_stop_state_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace spot_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EStopStateArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) spot_msgs::msg::EStopStateArray(_init);
}

void EStopStateArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<spot_msgs::msg::EStopStateArray *>(message_memory);
  typed_message->~EStopStateArray();
}

size_t size_function__EStopStateArray__estop_states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<spot_msgs::msg::EStopState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EStopStateArray__estop_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<spot_msgs::msg::EStopState> *>(untyped_member);
  return &member[index];
}

void * get_function__EStopStateArray__estop_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<spot_msgs::msg::EStopState> *>(untyped_member);
  return &member[index];
}

void fetch_function__EStopStateArray__estop_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const spot_msgs::msg::EStopState *>(
    get_const_function__EStopStateArray__estop_states(untyped_member, index));
  auto & value = *reinterpret_cast<spot_msgs::msg::EStopState *>(untyped_value);
  value = item;
}

void assign_function__EStopStateArray__estop_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<spot_msgs::msg::EStopState *>(
    get_function__EStopStateArray__estop_states(untyped_member, index));
  const auto & value = *reinterpret_cast<const spot_msgs::msg::EStopState *>(untyped_value);
  item = value;
}

void resize_function__EStopStateArray__estop_states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<spot_msgs::msg::EStopState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EStopStateArray_message_member_array[1] = {
  {
    "estop_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<spot_msgs::msg::EStopState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs::msg::EStopStateArray, estop_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__EStopStateArray__estop_states,  // size() function pointer
    get_const_function__EStopStateArray__estop_states,  // get_const(index) function pointer
    get_function__EStopStateArray__estop_states,  // get(index) function pointer
    fetch_function__EStopStateArray__estop_states,  // fetch(index, &value) function pointer
    assign_function__EStopStateArray__estop_states,  // assign(index, value) function pointer
    resize_function__EStopStateArray__estop_states  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EStopStateArray_message_members = {
  "spot_msgs::msg",  // message namespace
  "EStopStateArray",  // message name
  1,  // number of fields
  sizeof(spot_msgs::msg::EStopStateArray),
  EStopStateArray_message_member_array,  // message members
  EStopStateArray_init_function,  // function to initialize message memory (memory has to be allocated)
  EStopStateArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EStopStateArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EStopStateArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace spot_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::msg::EStopStateArray>()
{
  return &::spot_msgs::msg::rosidl_typesupport_introspection_cpp::EStopStateArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, msg, EStopStateArray)() {
  return &::spot_msgs::msg::rosidl_typesupport_introspection_cpp::EStopStateArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
