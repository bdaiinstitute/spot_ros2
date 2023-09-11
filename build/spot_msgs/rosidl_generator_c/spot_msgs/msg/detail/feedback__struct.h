// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'serial_number'
// Member 'species'
// Member 'version'
// Member 'nickname'
// Member 'computer_serial_number'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Feedback in the package spot_msgs.
typedef struct spot_msgs__msg__Feedback
{
  bool standing;
  bool sitting;
  bool moving;
  rosidl_runtime_c__String serial_number;
  rosidl_runtime_c__String species;
  rosidl_runtime_c__String version;
  rosidl_runtime_c__String nickname;
  rosidl_runtime_c__String computer_serial_number;
} spot_msgs__msg__Feedback;

// Struct for a sequence of spot_msgs__msg__Feedback.
typedef struct spot_msgs__msg__Feedback__Sequence
{
  spot_msgs__msg__Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
