// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from spot_msgs:srv/ListSounds.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "spot_msgs/srv/detail/list_sounds__rosidl_typesupport_introspection_c.h"
#include "spot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "spot_msgs/srv/detail/list_sounds__functions.h"
#include "spot_msgs/srv/detail/list_sounds__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  spot_msgs__srv__ListSounds_Request__init(message_memory);
}

void spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_fini_function(void * message_memory)
{
  spot_msgs__srv__ListSounds_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__srv__ListSounds_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_members = {
  "spot_msgs__srv",  // message namespace
  "ListSounds_Request",  // message name
  1,  // number of fields
  sizeof(spot_msgs__srv__ListSounds_Request),
  spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_member_array,  // message members
  spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_type_support_handle = {
  0,
  &spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_spot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds_Request)() {
  if (!spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_type_support_handle.typesupport_identifier) {
    spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &spot_msgs__srv__ListSounds_Request__rosidl_typesupport_introspection_c__ListSounds_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "spot_msgs/srv/detail/list_sounds__rosidl_typesupport_introspection_c.h"
// already included above
// #include "spot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "spot_msgs/srv/detail/list_sounds__functions.h"
// already included above
// #include "spot_msgs/srv/detail/list_sounds__struct.h"


// Include directives for member types
// Member `names`
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  spot_msgs__srv__ListSounds_Response__init(message_memory);
}

void spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_fini_function(void * message_memory)
{
  spot_msgs__srv__ListSounds_Response__fini(message_memory);
}

size_t spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__size_function__ListSounds_Response__names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__get_const_function__ListSounds_Response__names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__get_function__ListSounds_Response__names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__fetch_function__ListSounds_Response__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__get_const_function__ListSounds_Response__names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__assign_function__ListSounds_Response__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__get_function__ListSounds_Response__names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__resize_function__ListSounds_Response__names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_member_array[3] = {
  {
    "names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__srv__ListSounds_Response, names),  // bytes offset in struct
    NULL,  // default value
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__size_function__ListSounds_Response__names,  // size() function pointer
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__get_const_function__ListSounds_Response__names,  // get_const(index) function pointer
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__get_function__ListSounds_Response__names,  // get(index) function pointer
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__fetch_function__ListSounds_Response__names,  // fetch(index, &value) function pointer
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__assign_function__ListSounds_Response__names,  // assign(index, value) function pointer
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__resize_function__ListSounds_Response__names  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__srv__ListSounds_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(spot_msgs__srv__ListSounds_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_members = {
  "spot_msgs__srv",  // message namespace
  "ListSounds_Response",  // message name
  3,  // number of fields
  sizeof(spot_msgs__srv__ListSounds_Response),
  spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_member_array,  // message members
  spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_type_support_handle = {
  0,
  &spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_spot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds_Response)() {
  if (!spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_type_support_handle.typesupport_identifier) {
    spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &spot_msgs__srv__ListSounds_Response__rosidl_typesupport_introspection_c__ListSounds_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "spot_msgs/srv/detail/list_sounds__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_members = {
  "spot_msgs__srv",  // service namespace
  "ListSounds",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_Request_message_type_support_handle,
  NULL  // response message
  // spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_Response_message_type_support_handle
};

static rosidl_service_type_support_t spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_type_support_handle = {
  0,
  &spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_spot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds)() {
  if (!spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_type_support_handle.typesupport_identifier) {
    spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, srv, ListSounds_Response)()->data;
  }

  return &spot_msgs__srv__detail__list_sounds__rosidl_typesupport_introspection_c__ListSounds_service_type_support_handle;
}
