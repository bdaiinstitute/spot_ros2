// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/GetGripperCameraParameters.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GET_GRIPPER_CAMERA_PARAMETERS__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__GET_GRIPPER_CAMERA_PARAMETERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "bosdyn_msgs/msg/detail/gripper_camera_get_param_request__struct.h"

/// Struct defined in srv/GetGripperCameraParameters in the package spot_msgs.
typedef struct spot_msgs__srv__GetGripperCameraParameters_Request
{
  bosdyn_msgs__msg__GripperCameraGetParamRequest request;
} spot_msgs__srv__GetGripperCameraParameters_Request;

// Struct for a sequence of spot_msgs__srv__GetGripperCameraParameters_Request.
typedef struct spot_msgs__srv__GetGripperCameraParameters_Request__Sequence
{
  spot_msgs__srv__GetGripperCameraParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__GetGripperCameraParameters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "bosdyn_msgs/msg/detail/gripper_camera_get_param_response__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetGripperCameraParameters in the package spot_msgs.
typedef struct spot_msgs__srv__GetGripperCameraParameters_Response
{
  bosdyn_msgs__msg__GripperCameraGetParamResponse response;
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__srv__GetGripperCameraParameters_Response;

// Struct for a sequence of spot_msgs__srv__GetGripperCameraParameters_Response.
typedef struct spot_msgs__srv__GetGripperCameraParameters_Response__Sequence
{
  spot_msgs__srv__GetGripperCameraParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__GetGripperCameraParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__GET_GRIPPER_CAMERA_PARAMETERS__STRUCT_H_
