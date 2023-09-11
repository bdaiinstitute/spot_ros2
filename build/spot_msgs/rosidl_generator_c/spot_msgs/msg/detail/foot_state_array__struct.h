// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/FootStateArray.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FOOT_STATE_ARRAY__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__FOOT_STATE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'states'
#include "spot_msgs/msg/detail/foot_state__struct.h"

/// Struct defined in msg/FootStateArray in the package spot_msgs.
typedef struct spot_msgs__msg__FootStateArray
{
  spot_msgs__msg__FootState__Sequence states;
} spot_msgs__msg__FootStateArray;

// Struct for a sequence of spot_msgs__msg__FootStateArray.
typedef struct spot_msgs__msg__FootStateArray__Sequence
{
  spot_msgs__msg__FootStateArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__FootStateArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__FOOT_STATE_ARRAY__STRUCT_H_
