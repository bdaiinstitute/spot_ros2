// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from spot_msgs:srv/SetGripperCameraParameters.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__SET_GRIPPER_CAMERA_PARAMETERS__STRUCT_H_
#define SPOT_MSGS__SRV__DETAIL__SET_GRIPPER_CAMERA_PARAMETERS__STRUCT_H_

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
#include "bosdyn_msgs/msg/detail/gripper_camera_param_request__struct.h"

/// Struct defined in srv/SetGripperCameraParameters in the package spot_msgs.
typedef struct spot_msgs__srv__SetGripperCameraParameters_Request
{
  bosdyn_msgs__msg__GripperCameraParamRequest request;
} spot_msgs__srv__SetGripperCameraParameters_Request;

// Struct for a sequence of spot_msgs__srv__SetGripperCameraParameters_Request.
typedef struct spot_msgs__srv__SetGripperCameraParameters_Request__Sequence
{
  spot_msgs__srv__SetGripperCameraParameters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__SetGripperCameraParameters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "bosdyn_msgs/msg/detail/gripper_camera_param_response__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetGripperCameraParameters in the package spot_msgs.
typedef struct spot_msgs__srv__SetGripperCameraParameters_Response
{
  bosdyn_msgs__msg__GripperCameraParamResponse response;
  bool success;
  rosidl_runtime_c__String message;
} spot_msgs__srv__SetGripperCameraParameters_Response;

// Struct for a sequence of spot_msgs__srv__SetGripperCameraParameters_Response.
typedef struct spot_msgs__srv__SetGripperCameraParameters_Response__Sequence
{
  spot_msgs__srv__SetGripperCameraParameters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} spot_msgs__srv__SetGripperCameraParameters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SPOT_MSGS__SRV__DETAIL__SET_GRIPPER_CAMERA_PARAMETERS__STRUCT_H_
