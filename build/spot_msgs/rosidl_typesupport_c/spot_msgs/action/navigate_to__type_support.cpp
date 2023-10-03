// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from spot_msgs:action/NavigateTo.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "spot_msgs/action/detail/navigate_to__struct.h"
#include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_Goal_type_support_ids_t;

static const _NavigateTo_Goal_type_support_ids_t _NavigateTo_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_Goal_type_support_symbol_names_t _NavigateTo_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_Goal)),
  }
};

typedef struct _NavigateTo_Goal_type_support_data_t
{
  void * data[2];
} _NavigateTo_Goal_type_support_data_t;

static _NavigateTo_Goal_type_support_data_t _NavigateTo_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_Goal_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_Goal_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_Goal)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_Result_type_support_ids_t;

static const _NavigateTo_Result_type_support_ids_t _NavigateTo_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_Result_type_support_symbol_names_t _NavigateTo_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_Result)),
  }
};

typedef struct _NavigateTo_Result_type_support_data_t
{
  void * data[2];
} _NavigateTo_Result_type_support_data_t;

static _NavigateTo_Result_type_support_data_t _NavigateTo_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_Result_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_Result_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_Result_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_Result_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_Result)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_Feedback_type_support_ids_t;

static const _NavigateTo_Feedback_type_support_ids_t _NavigateTo_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_Feedback_type_support_symbol_names_t _NavigateTo_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_Feedback)),
  }
};

typedef struct _NavigateTo_Feedback_type_support_data_t
{
  void * data[2];
} _NavigateTo_Feedback_type_support_data_t;

static _NavigateTo_Feedback_type_support_data_t _NavigateTo_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_Feedback_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_Feedback_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_Feedback)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_SendGoal_Request_type_support_ids_t;

static const _NavigateTo_SendGoal_Request_type_support_ids_t _NavigateTo_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_SendGoal_Request_type_support_symbol_names_t _NavigateTo_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_SendGoal_Request)),
  }
};

typedef struct _NavigateTo_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _NavigateTo_SendGoal_Request_type_support_data_t;

static _NavigateTo_SendGoal_Request_type_support_data_t _NavigateTo_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_SendGoal_Request_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_SendGoal_Request_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_SendGoal_Request)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_SendGoal_Response_type_support_ids_t;

static const _NavigateTo_SendGoal_Response_type_support_ids_t _NavigateTo_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_SendGoal_Response_type_support_symbol_names_t _NavigateTo_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_SendGoal_Response)),
  }
};

typedef struct _NavigateTo_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _NavigateTo_SendGoal_Response_type_support_data_t;

static _NavigateTo_SendGoal_Response_type_support_data_t _NavigateTo_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_SendGoal_Response_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_SendGoal_Response_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_SendGoal_Response)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_SendGoal_type_support_ids_t;

static const _NavigateTo_SendGoal_type_support_ids_t _NavigateTo_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_SendGoal_type_support_symbol_names_t _NavigateTo_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_SendGoal)),
  }
};

typedef struct _NavigateTo_SendGoal_type_support_data_t
{
  void * data[2];
} _NavigateTo_SendGoal_type_support_data_t;

static _NavigateTo_SendGoal_type_support_data_t _NavigateTo_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_SendGoal_service_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t NavigateTo_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_SendGoal_service_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_SendGoal)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_GetResult_Request_type_support_ids_t;

static const _NavigateTo_GetResult_Request_type_support_ids_t _NavigateTo_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_GetResult_Request_type_support_symbol_names_t _NavigateTo_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_GetResult_Request)),
  }
};

typedef struct _NavigateTo_GetResult_Request_type_support_data_t
{
  void * data[2];
} _NavigateTo_GetResult_Request_type_support_data_t;

static _NavigateTo_GetResult_Request_type_support_data_t _NavigateTo_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_GetResult_Request_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_GetResult_Request_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_GetResult_Request)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_GetResult_Response_type_support_ids_t;

static const _NavigateTo_GetResult_Response_type_support_ids_t _NavigateTo_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_GetResult_Response_type_support_symbol_names_t _NavigateTo_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_GetResult_Response)),
  }
};

typedef struct _NavigateTo_GetResult_Response_type_support_data_t
{
  void * data[2];
} _NavigateTo_GetResult_Response_type_support_data_t;

static _NavigateTo_GetResult_Response_type_support_data_t _NavigateTo_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_GetResult_Response_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_GetResult_Response_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_GetResult_Response)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_GetResult_type_support_ids_t;

static const _NavigateTo_GetResult_type_support_ids_t _NavigateTo_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_GetResult_type_support_symbol_names_t _NavigateTo_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_GetResult)),
  }
};

typedef struct _NavigateTo_GetResult_type_support_data_t
{
  void * data[2];
} _NavigateTo_GetResult_type_support_data_t;

static _NavigateTo_GetResult_type_support_data_t _NavigateTo_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_GetResult_service_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t NavigateTo_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_GetResult_service_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_GetResult)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__struct.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"
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

typedef struct _NavigateTo_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NavigateTo_FeedbackMessage_type_support_ids_t;

static const _NavigateTo_FeedbackMessage_type_support_ids_t _NavigateTo_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NavigateTo_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NavigateTo_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NavigateTo_FeedbackMessage_type_support_symbol_names_t _NavigateTo_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, spot_msgs, action, NavigateTo_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, spot_msgs, action, NavigateTo_FeedbackMessage)),
  }
};

typedef struct _NavigateTo_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _NavigateTo_FeedbackMessage_type_support_data_t;

static _NavigateTo_FeedbackMessage_type_support_data_t _NavigateTo_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NavigateTo_FeedbackMessage_message_typesupport_map = {
  2,
  "spot_msgs",
  &_NavigateTo_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_NavigateTo_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_NavigateTo_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NavigateTo_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NavigateTo_FeedbackMessage_message_typesupport_map),
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, spot_msgs, action, NavigateTo_FeedbackMessage)() {
  return &::spot_msgs::action::rosidl_typesupport_c::NavigateTo_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "spot_msgs/action/navigate_to.h"
// already included above
// #include "spot_msgs/action/detail/navigate_to__type_support.h"

static rosidl_action_type_support_t _spot_msgs__action__NavigateTo__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, spot_msgs, action, NavigateTo)()
{
  // Thread-safe by always writing the same values to the static struct
  _spot_msgs__action__NavigateTo__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, spot_msgs, action, NavigateTo_SendGoal)();
  _spot_msgs__action__NavigateTo__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, spot_msgs, action, NavigateTo_GetResult)();
  _spot_msgs__action__NavigateTo__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _spot_msgs__action__NavigateTo__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, spot_msgs, action, NavigateTo_FeedbackMessage)();
  _spot_msgs__action__NavigateTo__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_spot_msgs__action__NavigateTo__typesupport_c;
}

#ifdef __cplusplus
}
#endif
