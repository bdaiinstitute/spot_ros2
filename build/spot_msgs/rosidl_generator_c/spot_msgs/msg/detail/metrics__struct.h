// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/Metrics.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__METRICS__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__METRICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'time_moving'
// Member 'electric_power'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/Metrics in the package spot_msgs.
typedef struct spot_msgs__msg__Metrics
{
  std_msgs__msg__Header header;
  float distance;
  int32_t gait_cycles;
  builtin_interfaces__msg__Duration time_moving;
  builtin_interfaces__msg__Duration electric_power;
} spot_msgs__msg__Metrics;

// Struct for a sequence of spot_msgs__msg__Metrics.
typedef struct spot_msgs__msg__Metrics__Sequence
{
  spot_msgs__msg__Metrics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__Metrics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__METRICS__STRUCT_H_
