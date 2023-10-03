// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from spot_msgs:msg/MobilityParams.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "spot_msgs/msg/detail/mobility_params__rosidl_typesupport_introspection_c.h"
#include "spot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "spot_msgs/msg/detail/mobility_params__functions.h"
#include "spot_msgs/msg/detail/mobility_params__struct.h"


// Include directives for member types
// Member `body_control`
#include "geometry_msgs/msg/pose.h"
// Member `body_control`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  spot_msgs__msg__MobilityParams__init(message_memory);
}

void spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_fini_function(void * message_memory)
{
  spot_msgs__msg__MobilityParams__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_member_array[3] = {
  {
    "body_control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__msg__MobilityParams, body_control),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "locomotion_hint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__msg__MobilityParams, locomotion_hint),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stair_hint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__msg__MobilityParams, stair_hint),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_members = {
  "spot_msgs__msg",  // message namespace
  "MobilityParams",  // message name
  3,  // number of fields
  sizeof(spot_msgs__msg__MobilityParams),
  spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_member_array,  // message members
  spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_init_function,  // function to initialize message memory (memory has to be allocated)
  spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_type_support_handle = {
  0,
  &spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_spot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, msg, MobilityParams)() {
  spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_type_support_handle.typesupport_identifier) {
    spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &spot_msgs__msg__MobilityParams__rosidl_typesupport_introspection_c__MobilityParams_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
