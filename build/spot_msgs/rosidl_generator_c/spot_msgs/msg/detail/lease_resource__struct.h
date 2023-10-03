// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:msg/LeaseResource.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__STRUCT_H_
#define SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'resource'
#include "rosidl_runtime_c/string.h"
// Member 'lease'
#include "spot_msgs/msg/detail/lease__struct.h"
// Member 'lease_owner'
#include "spot_msgs/msg/detail/lease_owner__struct.h"

/// Struct defined in msg/LeaseResource in the package spot_msgs.
typedef struct spot_msgs__msg__LeaseResource
{
  rosidl_runtime_c__String resource;
  spot_msgs__msg__Lease lease;
  spot_msgs__msg__LeaseOwner lease_owner;
} spot_msgs__msg__LeaseResource;

// Struct for a sequence of spot_msgs__msg__LeaseResource.
typedef struct spot_msgs__msg__LeaseResource__Sequence
{
  spot_msgs__msg__LeaseResource * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__msg__LeaseResource__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_RESOURCE__STRUCT_H_
