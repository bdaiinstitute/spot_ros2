// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/EStopStateArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__E_STOP_STATE_ARRAY__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__E_STOP_STATE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'estop_states'
#include "spot_msgs/msg/detail/e_stop_state__struct.h"

/// Struct defined in msg/EStopStateArray in the package spot_msgs.
typedef struct spot_msgs__msg__EStopStateArray
{
  spot_msgs__msg__EStopState__Sequence estop_states;
} spot_msgs__msg__EStopStateArray;

// Struct for a sequence of spot_msgs__msg__EStopStateArray.
typedef struct spot_msgs__msg__EStopStateArray__Sequence
{
  spot_msgs__msg__EStopStateArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__EStopStateArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__E_STOP_STATE_ARRAY__STRUCT_H_
