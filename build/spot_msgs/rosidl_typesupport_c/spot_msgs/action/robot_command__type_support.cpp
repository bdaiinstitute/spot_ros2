// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from spot_msgs:action/RobotCommand.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "spot_msgs/action/detail/robot_command__struct.h"
#include "spot_msgs/action/detail/robot_command__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_Goal_type_support_ids_t;

static const _RobotCommand_Goal_type_support_ids_t _RobotCommand_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_Goal_type_support_symbol_names_t _RobotCommand_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_Goal)),
  }
};

typedef struct _RobotCommand_Goal_type_support_data_t
{
  void * data[2];
} _RobotCommand_Goal_type_support_data_t;

static _RobotCommand_Goal_type_support_data_t _RobotCommand_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_Goal_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_Goal)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_Result_type_support_ids_t;

static const _RobotCommand_Result_type_support_ids_t _RobotCommand_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_Result_type_support_symbol_names_t _RobotCommand_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_Result)),
  }
};

typedef struct _RobotCommand_Result_type_support_data_t
{
  void * data[2];
} _RobotCommand_Result_type_support_data_t;

static _RobotCommand_Result_type_support_data_t _RobotCommand_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_Result_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_Result_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_Result_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_Result)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_Feedback_type_support_ids_t;

static const _RobotCommand_Feedback_type_support_ids_t _RobotCommand_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_Feedback_type_support_symbol_names_t _RobotCommand_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_Feedback)),
  }
};

typedef struct _RobotCommand_Feedback_type_support_data_t
{
  void * data[2];
} _RobotCommand_Feedback_type_support_data_t;

static _RobotCommand_Feedback_type_support_data_t _RobotCommand_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_Feedback_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_Feedback)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_SendGoal_Request_type_support_ids_t;

static const _RobotCommand_SendGoal_Request_type_support_ids_t _RobotCommand_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_SendGoal_Request_type_support_symbol_names_t _RobotCommand_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_SendGoal_Request)),
  }
};

typedef struct _RobotCommand_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _RobotCommand_SendGoal_Request_type_support_data_t;

static _RobotCommand_SendGoal_Request_type_support_data_t _RobotCommand_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_SendGoal_Request_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_SendGoal_Request)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_SendGoal_Response_type_support_ids_t;

static const _RobotCommand_SendGoal_Response_type_support_ids_t _RobotCommand_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_SendGoal_Response_type_support_symbol_names_t _RobotCommand_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_SendGoal_Response)),
  }
};

typedef struct _RobotCommand_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _RobotCommand_SendGoal_Response_type_support_data_t;

static _RobotCommand_SendGoal_Response_type_support_data_t _RobotCommand_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_SendGoal_Response_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_SendGoal_Response)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_SendGoal_type_support_ids_t;

static const _RobotCommand_SendGoal_type_support_ids_t _RobotCommand_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_SendGoal_type_support_symbol_names_t _RobotCommand_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_SendGoal)),
  }
};

typedef struct _RobotCommand_SendGoal_type_support_data_t
{
  void * data[2];
} _RobotCommand_SendGoal_type_support_data_t;

static _RobotCommand_SendGoal_type_support_data_t _RobotCommand_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_SendGoal_service_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RobotCommand_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_SendGoal)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_GetResult_Request_type_support_ids_t;

static const _RobotCommand_GetResult_Request_type_support_ids_t _RobotCommand_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_GetResult_Request_type_support_symbol_names_t _RobotCommand_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_GetResult_Request)),
  }
};

typedef struct _RobotCommand_GetResult_Request_type_support_data_t
{
  void * data[2];
} _RobotCommand_GetResult_Request_type_support_data_t;

static _RobotCommand_GetResult_Request_type_support_data_t _RobotCommand_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_GetResult_Request_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_GetResult_Request)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_GetResult_Response_type_support_ids_t;

static const _RobotCommand_GetResult_Response_type_support_ids_t _RobotCommand_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_GetResult_Response_type_support_symbol_names_t _RobotCommand_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_GetResult_Response)),
  }
};

typedef struct _RobotCommand_GetResult_Response_type_support_data_t
{
  void * data[2];
} _RobotCommand_GetResult_Response_type_support_data_t;

static _RobotCommand_GetResult_Response_type_support_data_t _RobotCommand_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_GetResult_Response_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_GetResult_Response)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_GetResult_type_support_ids_t;

static const _RobotCommand_GetResult_type_support_ids_t _RobotCommand_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_GetResult_type_support_symbol_names_t _RobotCommand_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_GetResult)),
  }
};

typedef struct _RobotCommand_GetResult_type_support_data_t
{
  void * data[2];
} _RobotCommand_GetResult_type_support_data_t;

static _RobotCommand_GetResult_type_support_data_t _RobotCommand_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_GetResult_service_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RobotCommand_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_GetResult)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _RobotCommand_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_FeedbackMessage_type_support_ids_t;

static const _RobotCommand_FeedbackMessage_type_support_ids_t _RobotCommand_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RobotCommand_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RobotCommand_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RobotCommand_FeedbackMessage_type_support_symbol_names_t _RobotCommand_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, RobotCommand_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, RobotCommand_FeedbackMessage)),
  }
};

typedef struct _RobotCommand_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _RobotCommand_FeedbackMessage_type_support_data_t;

static _RobotCommand_FeedbackMessage_type_support_data_t _RobotCommand_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RobotCommand_FeedbackMessage_message_typesupport_map = {
  2,
  "spot_msgs",
  &_RobotCommand_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_RobotCommand_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_RobotCommand_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RobotCommand_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace spot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, RobotCommand_FeedbackMessage)() {
  return &::spot_msgs::action::rosidl_typesupport_c::RobotCommand_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "spot_msgs/action/robot_command.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__type_support.h"

static rosidl_action_type_support_t _spot_msgs__action__RobotCommand__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, spot_msgs, action, RobotCommand)()
{
  // Thread-safe by always writing the same values to the static struct
  _spot_msgs__action__RobotCommand__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, spot_msgs, action, RobotCommand_SendGoal)();
  _spot_msgs__action__RobotCommand__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, spot_msgs, action, RobotCommand_GetResult)();
  _spot_msgs__action__RobotCommand__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _spot_msgs__action__RobotCommand__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, spot_msgs, action, RobotCommand_FeedbackMessage)();
  _spot_msgs__action__RobotCommand__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_spot_msgs__action__RobotCommand__typesupport_c;
}

#ifdef __cplusplus
}
#endif
