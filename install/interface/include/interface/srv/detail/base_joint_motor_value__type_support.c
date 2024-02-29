// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface:srv/BaseJointMotorValue.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface/srv/detail/base_joint_motor_value__rosidl_typesupport_introspection_c.h"
#include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface/srv/detail/base_joint_motor_value__functions.h"
#include "interface/srv/detail/base_joint_motor_value__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__srv__BaseJointMotorValue_Request__init(message_memory);
}

void BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_fini_function(void * message_memory)
{
  interface__srv__BaseJointMotorValue_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__BaseJointMotorValue_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_members = {
  "interface__srv",  // message namespace
  "BaseJointMotorValue_Request",  // message name
  1,  // number of fields
  sizeof(interface__srv__BaseJointMotorValue_Request),
  BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_member_array,  // message members
  BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_type_support_handle = {
  0,
  &BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue_Request)() {
  if (!BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_type_support_handle.typesupport_identifier) {
    BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BaseJointMotorValue_Request__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "interface/srv/detail/base_joint_motor_value__rosidl_typesupport_introspection_c.h"
// already included above
// #include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "interface/srv/detail/base_joint_motor_value__functions.h"
// already included above
// #include "interface/srv/detail/base_joint_motor_value__struct.h"


// Include directives for member types
// Member `cable_expend_value`
#include "geometry_msgs/msg/point.h"
// Member `cable_expend_value`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__srv__BaseJointMotorValue_Response__init(message_memory);
}

void BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_fini_function(void * message_memory)
{
  interface__srv__BaseJointMotorValue_Response__fini(message_memory);
}

size_t BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__size_function__Point__cable_expend_value(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__get_const_function__Point__cable_expend_value(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__get_function__Point__cable_expend_value(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

bool BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__resize_function__Point__cable_expend_value(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_member_array[4] = {
  {
    "cable_expend_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__BaseJointMotorValue_Response, cable_expend_value),  // bytes offset in struct
    NULL,  // default value
    BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__size_function__Point__cable_expend_value,  // size() function pointer
    BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__get_const_function__Point__cable_expend_value,  // get_const(index) function pointer
    BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__get_function__Point__cable_expend_value,  // get(index) function pointer
    BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__resize_function__Point__cable_expend_value  // resize(index) function pointer
  },
  {
    "base_advance_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__BaseJointMotorValue_Response, base_advance_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flag_base_out_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__BaseJointMotorValue_Response, flag_base_out_range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flag_arrived_target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__srv__BaseJointMotorValue_Response, flag_arrived_target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_members = {
  "interface__srv",  // message namespace
  "BaseJointMotorValue_Response",  // message name
  4,  // number of fields
  sizeof(interface__srv__BaseJointMotorValue_Response),
  BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_member_array,  // message members
  BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_type_support_handle = {
  0,
  &BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue_Response)() {
  BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_type_support_handle.typesupport_identifier) {
    BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BaseJointMotorValue_Response__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "interface/srv/detail/base_joint_motor_value__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_members = {
  "interface__srv",  // service namespace
  "BaseJointMotorValue",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_Request_message_type_support_handle,
  NULL  // response message
  // interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_Response_message_type_support_handle
};

static rosidl_service_type_support_t interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_type_support_handle = {
  0,
  &interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue)() {
  if (!interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_type_support_handle.typesupport_identifier) {
    interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, srv, BaseJointMotorValue_Response)()->data;
  }

  return &interface__srv__detail__base_joint_motor_value__rosidl_typesupport_introspection_c__BaseJointMotorValue_service_type_support_handle;
}
