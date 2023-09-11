// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from spot_msgs:msg/BehaviorFaultState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "spot_msgs/msg/detail/behavior_fault_state__rosidl_typesupport_introspection_c.h"
#include "spot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "spot_msgs/msg/detail/behavior_fault_state__functions.h"
#include "spot_msgs/msg/detail/behavior_fault_state__struct.h"


// Include directives for member types
// Member `faults`
#include "spot_msgs/msg/behavior_fault.h"
// Member `faults`
#include "spot_msgs/msg/detail/behavior_fault__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  spot_msgs__msg__BehaviorFaultState__init(message_memory);
}

void spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_fini_function(void * message_memory)
{
  spot_msgs__msg__BehaviorFaultState__fini(message_memory);
}

size_t spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__size_function__BehaviorFaultState__faults(
  const void * untyped_member)
{
  const spot_msgs__msg__BehaviorFault__Sequence * member =
    (const spot_msgs__msg__BehaviorFault__Sequence *)(untyped_member);
  return member->size;
}

const void * spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__get_const_function__BehaviorFaultState__faults(
  const void * untyped_member, size_t index)
{
  const spot_msgs__msg__BehaviorFault__Sequence * member =
    (const spot_msgs__msg__BehaviorFault__Sequence *)(untyped_member);
  return &member->data[index];
}

void * spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__get_function__BehaviorFaultState__faults(
  void * untyped_member, size_t index)
{
  spot_msgs__msg__BehaviorFault__Sequence * member =
    (spot_msgs__msg__BehaviorFault__Sequence *)(untyped_member);
  return &member->data[index];
}

void spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__fetch_function__BehaviorFaultState__faults(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const spot_msgs__msg__BehaviorFault * item =
    ((const spot_msgs__msg__BehaviorFault *)
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__get_const_function__BehaviorFaultState__faults(untyped_member, index));
  spot_msgs__msg__BehaviorFault * value =
    (spot_msgs__msg__BehaviorFault *)(untyped_value);
  *value = *item;
}

void spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__assign_function__BehaviorFaultState__faults(
  void * untyped_member, size_t index, const void * untyped_value)
{
  spot_msgs__msg__BehaviorFault * item =
    ((spot_msgs__msg__BehaviorFault *)
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__get_function__BehaviorFaultState__faults(untyped_member, index));
  const spot_msgs__msg__BehaviorFault * value =
    (const spot_msgs__msg__BehaviorFault *)(untyped_value);
  *item = *value;
}

bool spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__resize_function__BehaviorFaultState__faults(
  void * untyped_member, size_t size)
{
  spot_msgs__msg__BehaviorFault__Sequence * member =
    (spot_msgs__msg__BehaviorFault__Sequence *)(untyped_member);
  spot_msgs__msg__BehaviorFault__Sequence__fini(member);
  return spot_msgs__msg__BehaviorFault__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_member_array[1] = {
  {
    "faults",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__msg__BehaviorFaultState, faults),  // bytes offset in struct
    NULL,  // default value
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__size_function__BehaviorFaultState__faults,  // size() function pointer
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__get_const_function__BehaviorFaultState__faults,  // get_const(index) function pointer
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__get_function__BehaviorFaultState__faults,  // get(index) function pointer
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__fetch_function__BehaviorFaultState__faults,  // fetch(index, &value) function pointer
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__assign_function__BehaviorFaultState__faults,  // assign(index, value) function pointer
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__resize_function__BehaviorFaultState__faults  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_members = {
  "spot_msgs__msg",  // message namespace
  "BehaviorFaultState",  // message name
  1,  // number of fields
  sizeof(spot_msgs__msg__BehaviorFaultState),
  spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_member_array,  // message members
  spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_init_function,  // function to initialize message memory (memory has to be allocated)
  spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_type_support_handle = {
  0,
  &spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_spot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, msg, BehaviorFaultState)() {
  spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, msg, BehaviorFault)();
  if (!spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_type_support_handle.typesupport_identifier) {
    spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &spot_msgs__msg__BehaviorFaultState__rosidl_typesupport_introspection_c__BehaviorFaultState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
