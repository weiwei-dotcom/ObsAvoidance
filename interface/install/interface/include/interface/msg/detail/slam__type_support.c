// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface/msg/detail/slam__rosidl_typesupport_introspection_c.h"
#include "interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface/msg/detail/slam__functions.h"
#include "interface/msg/detail/slam__struct.h"


// Include directives for member types
// Member `point_cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"
// Member `cam_pose`
#include "geometry_msgs/msg/pose.h"
// Member `cam_pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `keypoints`
#include "geometry_msgs/msg/point32.h"
// Member `keypoints`
#include "geometry_msgs/msg/detail/point32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Slam__rosidl_typesupport_introspection_c__Slam_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface__msg__Slam__init(message_memory);
}

void Slam__rosidl_typesupport_introspection_c__Slam_fini_function(void * message_memory)
{
  interface__msg__Slam__fini(message_memory);
}

size_t Slam__rosidl_typesupport_introspection_c__size_function__Point32__keypoints(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point32__Sequence * member =
    (const geometry_msgs__msg__Point32__Sequence *)(untyped_member);
  return member->size;
}

const void * Slam__rosidl_typesupport_introspection_c__get_const_function__Point32__keypoints(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point32__Sequence * member =
    (const geometry_msgs__msg__Point32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Slam__rosidl_typesupport_introspection_c__get_function__Point32__keypoints(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point32__Sequence * member =
    (geometry_msgs__msg__Point32__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Slam__rosidl_typesupport_introspection_c__resize_function__Point32__keypoints(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point32__Sequence * member =
    (geometry_msgs__msg__Point32__Sequence *)(untyped_member);
  geometry_msgs__msg__Point32__Sequence__fini(member);
  return geometry_msgs__msg__Point32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Slam__rosidl_typesupport_introspection_c__Slam_message_member_array[3] = {
  {
    "point_cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__msg__Slam, point_cloud),  // bytes offset in struct
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
    offsetof(interface__msg__Slam, cam_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface__msg__Slam, keypoints),  // bytes offset in struct
    NULL,  // default value
    Slam__rosidl_typesupport_introspection_c__size_function__Point32__keypoints,  // size() function pointer
    Slam__rosidl_typesupport_introspection_c__get_const_function__Point32__keypoints,  // get_const(index) function pointer
    Slam__rosidl_typesupport_introspection_c__get_function__Point32__keypoints,  // get(index) function pointer
    Slam__rosidl_typesupport_introspection_c__resize_function__Point32__keypoints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Slam__rosidl_typesupport_introspection_c__Slam_message_members = {
  "interface__msg",  // message namespace
  "Slam",  // message name
  3,  // number of fields
  sizeof(interface__msg__Slam),
  Slam__rosidl_typesupport_introspection_c__Slam_message_member_array,  // message members
  Slam__rosidl_typesupport_introspection_c__Slam_init_function,  // function to initialize message memory (memory has to be allocated)
  Slam__rosidl_typesupport_introspection_c__Slam_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Slam__rosidl_typesupport_introspection_c__Slam_message_type_support_handle = {
  0,
  &Slam__rosidl_typesupport_introspection_c__Slam_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface, msg, Slam)() {
  Slam__rosidl_typesupport_introspection_c__Slam_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  Slam__rosidl_typesupport_introspection_c__Slam_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  Slam__rosidl_typesupport_introspection_c__Slam_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  if (!Slam__rosidl_typesupport_introspection_c__Slam_message_type_support_handle.typesupport_identifier) {
    Slam__rosidl_typesupport_introspection_c__Slam_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Slam__rosidl_typesupport_introspection_c__Slam_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
