// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from spot_msgs:msg/BehaviorFaultState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "spot_msgs/msg/detail/behavior_fault_state__struct.hpp"
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

void BehaviorFaultState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) spot_msgs::msg::BehaviorFaultState(_init);
}

void BehaviorFaultState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<spot_msgs::msg::BehaviorFaultState *>(message_memory);
  typed_message->~BehaviorFaultState();
}

size_t size_function__BehaviorFaultState__faults(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<spot_msgs::msg::BehaviorFault> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BehaviorFaultState__faults(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<spot_msgs::msg::BehaviorFault> *>(untyped_member);
  return &member[index];
}

void * get_function__BehaviorFaultState__faults(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<spot_msgs::msg::BehaviorFault> *>(untyped_member);
  return &member[index];
}

void fetch_function__BehaviorFaultState__faults(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const spot_msgs::msg::BehaviorFault *>(
    get_const_function__BehaviorFaultState__faults(untyped_member, index));
  auto & value = *reinterpret_cast<spot_msgs::msg::BehaviorFault *>(untyped_value);
  value = item;
}

void assign_function__BehaviorFaultState__faults(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<spot_msgs::msg::BehaviorFault *>(
    get_function__BehaviorFaultState__faults(untyped_member, index));
  const auto & value = *reinterpret_cast<const spot_msgs::msg::BehaviorFault *>(untyped_value);
  item = value;
}

void resize_function__BehaviorFaultState__faults(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<spot_msgs::msg::BehaviorFault> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BehaviorFaultState_message_member_array[1] = {
  {
    "faults",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<spot_msgs::msg::BehaviorFault>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs::msg::BehaviorFaultState, faults),  // bytes offset in struct
    nullptr,  // default value
    size_function__BehaviorFaultState__faults,  // size() function pointer
    get_const_function__BehaviorFaultState__faults,  // get_const(index) function pointer
    get_function__BehaviorFaultState__faults,  // get(index) function pointer
    fetch_function__BehaviorFaultState__faults,  // fetch(index, &value) function pointer
    assign_function__BehaviorFaultState__faults,  // assign(index, value) function pointer
    resize_function__BehaviorFaultState__faults  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BehaviorFaultState_message_members = {
  "spot_msgs::msg",  // message namespace
  "BehaviorFaultState",  // message name
  1,  // number of fields
  sizeof(spot_msgs::msg::BehaviorFaultState),
  BehaviorFaultState_message_member_array,  // message members
  BehaviorFaultState_init_function,  // function to initialize message memory (memory has to be allocated)
  BehaviorFaultState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BehaviorFaultState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BehaviorFaultState_message_members,
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
get_message_type_support_handle<spot_msgs::msg::BehaviorFaultState>()
{
  return &::spot_msgs::msg::rosidl_typesupport_introspection_cpp::BehaviorFaultState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, msg, BehaviorFaultState)() {
  return &::spot_msgs::msg::rosidl_typesupport_introspection_cpp::BehaviorFaultState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
