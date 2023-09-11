// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/SystemFaultState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT_STATE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'faults'
// Member 'historical_faults'
#include "spot_msgs/msg/detail/system_fault__struct.h"

/// Struct defined in msg/SystemFaultState in the package spot_msgs.
typedef struct spot_msgs__msg__SystemFaultState
{
  spot_msgs__msg__SystemFault__Sequence faults;
  spot_msgs__msg__SystemFault__Sequence historical_faults;
} spot_msgs__msg__SystemFaultState;

// Struct for a sequence of spot_msgs__msg__SystemFaultState.
typedef struct spot_msgs__msg__SystemFaultState__Sequence
{
  spot_msgs__msg__SystemFaultState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__SystemFaultState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT_STATE__STRUCT_H_
