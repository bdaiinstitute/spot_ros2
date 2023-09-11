// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/PowerState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__POWER_STATE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__POWER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_UNKNOWN'.
enum
{
  spot_msgs__msg__PowerState__STATE_UNKNOWN = 0
};

/// Constant 'STATE_OFF'.
enum
{
  spot_msgs__msg__PowerState__STATE_OFF = 1
};

/// Constant 'STATE_ON'.
enum
{
  spot_msgs__msg__PowerState__STATE_ON = 2
};

/// Constant 'STATE_POWERING_ON'.
enum
{
  spot_msgs__msg__PowerState__STATE_POWERING_ON = 3
};

/// Constant 'STATE_POWERING_OFF'.
enum
{
  spot_msgs__msg__PowerState__STATE_POWERING_OFF = 4
};

/// Constant 'STATE_ERROR'.
enum
{
  spot_msgs__msg__PowerState__STATE_ERROR = 5
};

/// Constant 'STATE_UNKNOWN_SHORE_POWER'.
/**
  * ShorePowerState
 */
enum
{
  spot_msgs__msg__PowerState__STATE_UNKNOWN_SHORE_POWER = 0
};

/// Constant 'STATE_ON_SHORE_POWER'.
enum
{
  spot_msgs__msg__PowerState__STATE_ON_SHORE_POWER = 1
};

/// Constant 'STATE_OFF_SHORE_POWER'.
enum
{
  spot_msgs__msg__PowerState__STATE_OFF_SHORE_POWER = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'locomotion_estimated_runtime'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/PowerState in the package spot_msgs.
/**
  * MotorPowerState
 */
typedef struct spot_msgs__msg__PowerState
{
  std_msgs__msg__Header header;
  uint8_t motor_power_state;
  uint8_t shore_power_state;
  double locomotion_charge_percentage;
  builtin_interfaces__msg__Duration locomotion_estimated_runtime;
} spot_msgs__msg__PowerState;

// Struct for a sequence of spot_msgs__msg__PowerState.
typedef struct spot_msgs__msg__PowerState__Sequence
{
  spot_msgs__msg__PowerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__PowerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__POWER_STATE__STRUCT_H_
