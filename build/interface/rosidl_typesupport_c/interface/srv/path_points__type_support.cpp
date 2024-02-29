// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "interface/msg/rosidl_typesupport_c__visibility_control.h"
#include "interface/srv/detail/path_points__struct.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace interface
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PathPoints_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPoints_Request_type_support_ids_t;

static const _PathPoints_Request_type_support_ids_t _PathPoints_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PathPoints_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPoints_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPoints_Request_type_support_symbol_names_t _PathPoints_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, PathPoints_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Request)),
  }
};

typedef struct _PathPoints_Request_type_support_data_t
{
  void * data[2];
} _PathPoints_Request_type_support_data_t;

static _PathPoints_Request_type_support_data_t _PathPoints_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPoints_Request_message_typesupport_map = {
  2,
  "interface",
  &_PathPoints_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PathPoints_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PathPoints_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPoints_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPoints_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, interface, srv, PathPoints_Request)() {
  return &::interface::srv::rosidl_typesupport_c::PathPoints_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "interface/msg/rosidl_typesupport_c__visibility_control.h"
// already included above
// #include "interface/srv/detail/path_points__struct.h"
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

namespace interface
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PathPoints_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPoints_Response_type_support_ids_t;

static const _PathPoints_Response_type_support_ids_t _PathPoints_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PathPoints_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPoints_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPoints_Response_type_support_symbol_names_t _PathPoints_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, PathPoints_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Response)),
  }
};

typedef struct _PathPoints_Response_type_support_data_t
{
  void * data[2];
} _PathPoints_Response_type_support_data_t;

static _PathPoints_Response_type_support_data_t _PathPoints_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPoints_Response_message_typesupport_map = {
  2,
  "interface",
  &_PathPoints_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PathPoints_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PathPoints_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathPoints_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPoints_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, interface, srv, PathPoints_Response)() {
  return &::interface::srv::rosidl_typesupport_c::PathPoints_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "interface/msg/rosidl_typesupport_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace interface
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PathPoints_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathPoints_type_support_ids_t;

static const _PathPoints_type_support_ids_t _PathPoints_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PathPoints_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathPoints_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathPoints_type_support_symbol_names_t _PathPoints_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, PathPoints)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints)),
  }
};

typedef struct _PathPoints_type_support_data_t
{
  void * data[2];
} _PathPoints_type_support_data_t;

static _PathPoints_type_support_data_t _PathPoints_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathPoints_service_typesupport_map = {
  2,
  "interface",
  &_PathPoints_service_typesupport_ids.typesupport_identifier[0],
  &_PathPoints_service_typesupport_symbol_names.symbol_name[0],
  &_PathPoints_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PathPoints_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathPoints_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_C_EXPORT_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, interface, srv, PathPoints)() {
  return &::interface::srv::rosidl_typesupport_c::PathPoints_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
