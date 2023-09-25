// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface:srv/MapPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface/srv/detail/map_point__rosidl_typesupport_introspection_c.h"
#include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface/srv/detail/map_point__functions.h"
#include "interface/srv/detail/map_point__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__srv__MapPoint_Request__init(message_memory);
}

void MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_fini_function(void * message_memory)
{
  interface__srv__MapPoint_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__MapPoint_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_members = {
  "interface__srv",  // message namespace
  "MapPoint_Request",  // message name
  1,  // number of fields
  sizeof(interface__srv__MapPoint_Request),
  MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_member_array,  // message members
  MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_type_support_handle = {
  0,
  &MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint_Request)() {
  if (!MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_type_support_handle.typesupport_identifier) {
    MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MapPoint_Request__rosidl_typesupport_introspection_c__MapPoint_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "interface/srv/detail/map_point__rosidl_typesupport_introspection_c.h"
// already included above
// #include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "interface/srv/detail/map_point__functions.h"
// already included above
// #include "interface/srv/detail/map_point__struct.h"


// Include directives for member types
// Member `point_cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"
// Member `img`
#include "sensor_msgs/msg/image.h"
// Member `img`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `cam_pose`
// Member `world2cam`
#include "geometry_msgs/msg/pose.h"
// Member `cam_pose`
// Member `world2cam`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__srv__MapPoint_Response__init(message_memory);
}

void MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_fini_function(void * message_memory)
{
  interface__srv__MapPoint_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_member_array[4] = {
  {
    "point_cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__MapPoint_Response, point_cloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "img",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__MapPoint_Response, img),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cam_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__MapPoint_Response, cam_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "world2cam",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__MapPoint_Response, world2cam),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_members = {
  "interface__srv",  // message namespace
  "MapPoint_Response",  // message name
  4,  // number of fields
  sizeof(interface__srv__MapPoint_Response),
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_member_array,  // message members
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_type_support_handle = {
  0,
  &MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint_Response)() {
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_type_support_handle.typesupport_identifier) {
    MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MapPoint_Response__rosidl_typesupport_introspection_c__MapPoint_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "interface/srv/detail/map_point__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_members = {
  "interface__srv",  // service namespace
  "MapPoint",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_Request_message_type_support_handle,
  NULL  // response message
  // interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_Response_message_type_support_handle
};

static rosidl_service_type_support_t interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_type_support_handle = {
  0,
  &interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint)() {
  if (!interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_type_support_handle.typesupport_identifier) {
    interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, MapPoint_Response)()->data;
  }

  return &interface__srv__detail__map_point__rosidl_typesupport_introspection_c__MapPoint_service_type_support_handle;
}
