// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface/srv/detail/path_points__rosidl_typesupport_introspection_c.h"
#include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface/srv/detail/path_points__functions.h"
#include "interface/srv/detail/path_points__struct.h"


// Include directives for member types
// Member `start_position`
#include "geometry_msgs/msg/point.h"
// Member `start_position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `start_speed`
#include "geometry_msgs/msg/vector3.h"
// Member `start_speed`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__srv__PathPoints_Request__init(message_memory);
}

void PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_fini_function(void * message_memory)
{
  interface__srv__PathPoints_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_member_array[2] = {
  {
    "start_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__PathPoints_Request, start_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__PathPoints_Request, start_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_members = {
  "interface__srv",  // message namespace
  "PathPoints_Request",  // message name
  2,  // number of fields
  sizeof(interface__srv__PathPoints_Request),
  PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_member_array,  // message members
  PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_type_support_handle = {
  0,
  &PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Request)() {
  PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_type_support_handle.typesupport_identifier) {
    PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PathPoints_Request__rosidl_typesupport_introspection_c__PathPoints_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "interface/srv/detail/path_points__rosidl_typesupport_introspection_c.h"
// already included above
// #include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "interface/srv/detail/path_points__functions.h"
// already included above
// #include "interface/srv/detail/path_points__struct.h"


// Include directives for member types
// Member `path_points`
// already included above
// #include "geometry_msgs/msg/point.h"
// Member `path_points`
// already included above
// #include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__srv__PathPoints_Response__init(message_memory);
}

void PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_fini_function(void * message_memory)
{
  interface__srv__PathPoints_Response__fini(message_memory);
}

size_t PathPoints_Response__rosidl_typesupport_introspection_c__size_function__Point__path_points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * PathPoints_Response__rosidl_typesupport_introspection_c__get_const_function__Point__path_points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * PathPoints_Response__rosidl_typesupport_introspection_c__get_function__Point__path_points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

bool PathPoints_Response__rosidl_typesupport_introspection_c__resize_function__Point__path_points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_member_array[2] = {
  {
    "path_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__PathPoints_Response, path_points),  // bytes offset in struct
    NULL,  // default value
    PathPoints_Response__rosidl_typesupport_introspection_c__size_function__Point__path_points,  // size() function pointer
    PathPoints_Response__rosidl_typesupport_introspection_c__get_const_function__Point__path_points,  // get_const(index) function pointer
    PathPoints_Response__rosidl_typesupport_introspection_c__get_function__Point__path_points,  // get(index) function pointer
    PathPoints_Response__rosidl_typesupport_introspection_c__resize_function__Point__path_points  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__PathPoints_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_members = {
  "interface__srv",  // message namespace
  "PathPoints_Response",  // message name
  2,  // number of fields
  sizeof(interface__srv__PathPoints_Response),
  PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_member_array,  // message members
  PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_type_support_handle = {
  0,
  &PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Response)() {
  PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_type_support_handle.typesupport_identifier) {
    PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PathPoints_Response__rosidl_typesupport_introspection_c__PathPoints_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "interface/srv/detail/path_points__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_members = {
  "interface__srv",  // service namespace
  "PathPoints",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_Request_message_type_support_handle,
  NULL  // response message
  // interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_Response_message_type_support_handle
};

static rosidl_service_type_support_t interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_type_support_handle = {
  0,
  &interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints)() {
  if (!interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_type_support_handle.typesupport_identifier) {
    interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, PathPoints_Response)()->data;
  }

  return &interface__srv__detail__path_points__rosidl_typesupport_introspection_c__PathPoints_service_type_support_handle;
}
