// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/MobilityParams.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'body_control'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/MobilityParams in the package spot_msgs.
typedef struct spot_msgs__msg__MobilityParams
{
  geometry_msgs__msg__Pose body_control;
  uint32_t locomotion_hint;
  bool stair_hint;
} spot_msgs__msg__MobilityParams;

// Struct for a sequence of spot_msgs__msg__MobilityParams.
typedef struct spot_msgs__msg__MobilityParams__Sequence
{
  spot_msgs__msg__MobilityParams * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__MobilityParams__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__STRUCT_H_
