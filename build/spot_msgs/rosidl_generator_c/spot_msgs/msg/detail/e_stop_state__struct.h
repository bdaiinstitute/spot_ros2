// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/EStopState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__E_STOP_STATE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__E_STOP_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_UNKNOWN'.
enum
{
  spot_msgs__msg__EStopState__TYPE_UNKNOWN = 0
};

/// Constant 'TYPE_HARDWARE'.
enum
{
  spot_msgs__msg__EStopState__TYPE_HARDWARE = 1
};

/// Constant 'TYPE_SOFTWARE'.
enum
{
  spot_msgs__msg__EStopState__TYPE_SOFTWARE = 2
};

/// Constant 'STATE_UNKNOWN'.
/**
  * State
 */
enum
{
  spot_msgs__msg__EStopState__STATE_UNKNOWN = 0
};

/// Constant 'STATE_ESTOPPED'.
enum
{
  spot_msgs__msg__EStopState__STATE_ESTOPPED = 1
};

/// Constant 'STATE_NOT_ESTOPPED'.
enum
{
  spot_msgs__msg__EStopState__STATE_NOT_ESTOPPED = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
// Member 'state_description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EStopState in the package spot_msgs.
/**
  * Type
 */
typedef struct spot_msgs__msg__EStopState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String name;
  uint8_t type;
  uint8_t state;
  rosidl_runtime_c__String state_description;
} spot_msgs__msg__EStopState;

// Struct for a sequence of spot_msgs__msg__EStopState.
typedef struct spot_msgs__msg__EStopState__Sequence
{
  spot_msgs__msg__EStopState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__EStopState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__E_STOP_STATE__STRUCT_H_
