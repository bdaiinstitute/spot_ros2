// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/WiFiState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__WI_FI_STATE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__WI_FI_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_UNKNOWN'.
enum
{
  spot_msgs__msg__WiFiState__MODE_UNKNOWN = 0
};

/// Constant 'MODE_ACCESS_POINT'.
enum
{
  spot_msgs__msg__WiFiState__MODE_ACCESS_POINT = 1
};

/// Constant 'MODE_CLIENT'.
enum
{
  spot_msgs__msg__WiFiState__MODE_CLIENT = 2
};

// Include directives for member types
// Member 'essid'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/WiFiState in the package spot_msgs.
/**
  * Mode
 */
typedef struct spot_msgs__msg__WiFiState
{
  uint8_t current_mode;
  rosidl_runtime_c__String essid;
} spot_msgs__msg__WiFiState;

// Struct for a sequence of spot_msgs__msg__WiFiState.
typedef struct spot_msgs__msg__WiFiState__Sequence
{
  spot_msgs__msg__WiFiState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__WiFiState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__WI_FI_STATE__STRUCT_H_
