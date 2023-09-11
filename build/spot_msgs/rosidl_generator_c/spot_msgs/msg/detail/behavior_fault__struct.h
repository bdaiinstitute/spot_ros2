// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/BehaviorFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CAUSE_UNKNOWN'.
enum
{
  spot_msgs__msg__BehaviorFault__CAUSE_UNKNOWN = 0
};

/// Constant 'CAUSE_FALL'.
enum
{
  spot_msgs__msg__BehaviorFault__CAUSE_FALL = 1
};

/// Constant 'CAUSE_HARDWARE'.
enum
{
  spot_msgs__msg__BehaviorFault__CAUSE_HARDWARE = 2
};

/// Constant 'STATUS_UNKNOWN'.
/**
  * Status
 */
enum
{
  spot_msgs__msg__BehaviorFault__STATUS_UNKNOWN = 0
};

/// Constant 'STATUS_CLEARABLE'.
enum
{
  spot_msgs__msg__BehaviorFault__STATUS_CLEARABLE = 1
};

/// Constant 'STATUS_UNCLEARABLE'.
enum
{
  spot_msgs__msg__BehaviorFault__STATUS_UNCLEARABLE = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/BehaviorFault in the package spot_msgs.
/**
  * Cause
 */
typedef struct spot_msgs__msg__BehaviorFault
{
  std_msgs__msg__Header header;
  uint32_t behavior_fault_id;
  uint8_t cause;
  uint8_t status;
} spot_msgs__msg__BehaviorFault;

// Struct for a sequence of spot_msgs__msg__BehaviorFault.
typedef struct spot_msgs__msg__BehaviorFault__Sequence
{
  spot_msgs__msg__BehaviorFault * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__BehaviorFault__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__STRUCT_H_
