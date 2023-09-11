// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/BatteryState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BATTERY_STATE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__BATTERY_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_UNKNOWN'.
enum
{
  spot_msgs__msg__BatteryState__STATUS_UNKNOWN = 0
};

/// Constant 'STATUS_MISSING'.
enum
{
  spot_msgs__msg__BatteryState__STATUS_MISSING = 1
};

/// Constant 'STATUS_CHARGING'.
enum
{
  spot_msgs__msg__BatteryState__STATUS_CHARGING = 2
};

/// Constant 'STATUS_DISCHARGING'.
enum
{
  spot_msgs__msg__BatteryState__STATUS_DISCHARGING = 3
};

/// Constant 'STATUS_BOOTING'.
enum
{
  spot_msgs__msg__BatteryState__STATUS_BOOTING = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'identifier'
#include "rosidl_runtime_c/string.h"
// Member 'estimated_runtime'
#include "builtin_interfaces/msg/detail/duration__struct.h"
// Member 'temperatures'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BatteryState in the package spot_msgs.
/**
  * Status
 */
typedef struct spot_msgs__msg__BatteryState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String identifier;
  double charge_percentage;
  builtin_interfaces__msg__Duration estimated_runtime;
  double current;
  double voltage;
  rosidl_runtime_c__double__Sequence temperatures;
  uint8_t status;
} spot_msgs__msg__BatteryState;

// Struct for a sequence of spot_msgs__msg__BatteryState.
typedef struct spot_msgs__msg__BatteryState__Sequence
{
  spot_msgs__msg__BatteryState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__BatteryState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__BATTERY_STATE__STRUCT_H_
