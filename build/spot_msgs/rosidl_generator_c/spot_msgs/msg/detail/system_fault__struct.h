// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/SystemFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SEVERITY_UNKNOWN'.
enum
{
  spot_msgs__msg__SystemFault__SEVERITY_UNKNOWN = 0
};

/// Constant 'SEVERITY_INFO'.
enum
{
  spot_msgs__msg__SystemFault__SEVERITY_INFO = 1
};

/// Constant 'SEVERITY_WARN'.
enum
{
  spot_msgs__msg__SystemFault__SEVERITY_WARN = 2
};

/// Constant 'SEVERITY_CRITICAL'.
enum
{
  spot_msgs__msg__SystemFault__SEVERITY_CRITICAL = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
// Member 'error_message'
// Member 'attributes'
#include "rosidl_runtime_c/string.h"
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/SystemFault in the package spot_msgs.
/**
  * Severity
 */
typedef struct spot_msgs__msg__SystemFault
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String name;
  builtin_interfaces__msg__Duration duration;
  int32_t code;
  uint64_t uid;
  rosidl_runtime_c__String error_message;
  rosidl_runtime_c__String__Sequence attributes;
  uint8_t severity;
} spot_msgs__msg__SystemFault;

// Struct for a sequence of spot_msgs__msg__SystemFault.
typedef struct spot_msgs__msg__SystemFault__Sequence
{
  spot_msgs__msg__SystemFault * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__SystemFault__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__STRUCT_H_
