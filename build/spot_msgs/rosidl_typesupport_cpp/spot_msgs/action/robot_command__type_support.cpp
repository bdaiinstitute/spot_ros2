// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from spot_msgs:action/RobotCommand.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "spot_msgs/action/detail/robot_command__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_Goal_type_support_ids_t;

static const _RobotCommand_Goal_type_support_ids_t _RobotCommand_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_Goal)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_Goal>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_Goal)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_Result_type_support_ids_t;

static const _RobotCommand_Result_type_support_ids_t _RobotCommand_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_Result)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_Result>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_Result)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_Feedback_type_support_ids_t;

static const _RobotCommand_Feedback_type_support_ids_t _RobotCommand_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_Feedback)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_Feedback>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_Feedback)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_SendGoal_Request_type_support_ids_t;

static const _RobotCommand_SendGoal_Request_type_support_ids_t _RobotCommand_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_SendGoal_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_SendGoal_Request>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_SendGoal_Request)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_SendGoal_Response_type_support_ids_t;

static const _RobotCommand_SendGoal_Response_type_support_ids_t _RobotCommand_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_SendGoal_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_SendGoal_Response>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_SendGoal_Response)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_SendGoal_type_support_ids_t;

static const _RobotCommand_SendGoal_type_support_ids_t _RobotCommand_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_SendGoal)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<spot_msgs::action::RobotCommand_SendGoal>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_GetResult_Request_type_support_ids_t;

static const _RobotCommand_GetResult_Request_type_support_ids_t _RobotCommand_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_GetResult_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_GetResult_Request>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_GetResult_Request)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_GetResult_Response_type_support_ids_t;

static const _RobotCommand_GetResult_Response_type_support_ids_t _RobotCommand_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_GetResult_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_GetResult_Response>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_GetResult_Response)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_GetResult_type_support_ids_t;

static const _RobotCommand_GetResult_type_support_ids_t _RobotCommand_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_GetResult)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<spot_msgs::action::RobotCommand_GetResult>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _RobotCommand_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RobotCommand_FeedbackMessage_type_support_ids_t;

static const _RobotCommand_FeedbackMessage_type_support_ids_t _RobotCommand_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, spot_msgs, action, RobotCommand_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, spot_msgs, action, RobotCommand_FeedbackMessage)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RobotCommand_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<spot_msgs::action::RobotCommand_FeedbackMessage>()
{
  return &::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, spot_msgs, action, RobotCommand_FeedbackMessage)() {
  return get_message_type_support_handle<spot_msgs::action::RobotCommand_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "spot_msgs/action/detail/robot_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace spot_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t RobotCommand_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace spot_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<spot_msgs::action::RobotCommand>()
{
  using ::spot_msgs::action::rosidl_typesupport_cpp::RobotCommand_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  RobotCommand_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::spot_msgs::action::RobotCommand::Impl::SendGoalService>();
  RobotCommand_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::spot_msgs::action::RobotCommand::Impl::GetResultService>();
  RobotCommand_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::spot_msgs::action::RobotCommand::Impl::CancelGoalService>();
  RobotCommand_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::spot_msgs::action::RobotCommand::Impl::FeedbackMessage>();
  RobotCommand_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::spot_msgs::action::RobotCommand::Impl::GoalStatusMessage>();
  return &RobotCommand_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
