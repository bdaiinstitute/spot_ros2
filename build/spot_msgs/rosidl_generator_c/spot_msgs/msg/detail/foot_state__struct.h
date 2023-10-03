// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CONTACT_UNKNOWN'.
enum
{
  spot_msgs__msg__FootState__CONTACT_UNKNOWN = 0
};

/// Constant 'CONTACT_MADE'.
enum
{
  spot_msgs__msg__FootState__CONTACT_MADE = 1
};

/// Constant 'CONTACT_LOST'.
enum
{
  spot_msgs__msg__FootState__CONTACT_LOST = 2
};

// Include directives for member types
// Member 'foot_position_rt_body'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/FootState in the package spot_msgs.
/**
  * Contact
 */
typedef struct spot_msgs__msg__FootState
{
  geometry_msgs__msg__Point foot_position_rt_body;
  uint8_t contact;
} spot_msgs__msg__FootState;

// Struct for a sequence of spot_msgs__msg__FootState.
typedef struct spot_msgs__msg__FootState__Sequence
{
  spot_msgs__msg__FootState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__FootState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_
