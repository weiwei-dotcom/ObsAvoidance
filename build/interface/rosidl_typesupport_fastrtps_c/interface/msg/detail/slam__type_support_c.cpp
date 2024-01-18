// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice
#include "interface/msg/detail/slam__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface/msg/detail/slam__struct.h"
#include "interface/msg/detail/slam__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

<<<<<<< HEAD
#include "geometry_msgs/msg/detail/pose__functions.h"  // cam_pose, world2cam
#include "sensor_msgs/msg/detail/image__functions.h"  // img
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"  // point_cloud

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_geometry_msgs__msg__Pose(
=======
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // transform_init2cur
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"  // point_cloud
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_geometry_msgs__msg__PoseStamped(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
<<<<<<< HEAD
size_t max_serialized_size_geometry_msgs__msg__Pose(
=======
size_t max_serialized_size_geometry_msgs__msg__PoseStamped(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
const rosidl_message_type_support_t *
<<<<<<< HEAD
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_sensor_msgs__msg__Image(
=======
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_sensor_msgs__msg__PointCloud2(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
<<<<<<< HEAD
size_t max_serialized_size_sensor_msgs__msg__Image(
=======
size_t max_serialized_size_sensor_msgs__msg__PointCloud2(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
const rosidl_message_type_support_t *
<<<<<<< HEAD
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_sensor_msgs__msg__PointCloud2(
=======
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_std_msgs__msg__Header(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
<<<<<<< HEAD
size_t max_serialized_size_sensor_msgs__msg__PointCloud2(
=======
size_t max_serialized_size_std_msgs__msg__Header(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
const rosidl_message_type_support_t *
<<<<<<< HEAD
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2)();
=======
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09


using _Slam__ros_msg_type = interface__msg__Slam;

static bool _Slam__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Slam__ros_msg_type * ros_message = static_cast<const _Slam__ros_msg_type *>(untyped_ros_message);
<<<<<<< HEAD
  // Field name: point_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->point_cloud, cdr))
    {
      return false;
    }
  }

  // Field name: cam_pose
=======
  // Field name: header
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
<<<<<<< HEAD
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->cam_pose, cdr))
=======
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    {
      return false;
    }
  }

<<<<<<< HEAD
  // Field name: world2cam
=======
  // Field name: point_cloud
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
<<<<<<< HEAD
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->world2cam, cdr))
=======
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->point_cloud, cdr))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    {
      return false;
    }
  }

<<<<<<< HEAD
  // Field name: img
=======
  // Field name: transform_init2cur
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
<<<<<<< HEAD
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->img, cdr))
=======
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->transform_init2cur, cdr))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    {
      return false;
    }
  }

  return true;
}

static bool _Slam__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Slam__ros_msg_type * ros_message = static_cast<_Slam__ros_msg_type *>(untyped_ros_message);
<<<<<<< HEAD
  // Field name: point_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->point_cloud))
    {
      return false;
    }
  }

  // Field name: cam_pose
=======
  // Field name: header
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
<<<<<<< HEAD
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->cam_pose))
=======
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    {
      return false;
    }
  }

<<<<<<< HEAD
  // Field name: world2cam
=======
  // Field name: point_cloud
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
<<<<<<< HEAD
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->world2cam))
=======
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->point_cloud))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    {
      return false;
    }
  }

<<<<<<< HEAD
  // Field name: img
=======
  // Field name: transform_init2cur
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
<<<<<<< HEAD
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->img))
=======
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->transform_init2cur))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t get_serialized_size_interface__msg__Slam(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Slam__ros_msg_type * ros_message = static_cast<const _Slam__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

<<<<<<< HEAD
=======
  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  // field.name point_cloud

  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->point_cloud), current_alignment);
<<<<<<< HEAD
  // field.name cam_pose

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->cam_pose), current_alignment);
  // field.name world2cam

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->world2cam), current_alignment);
  // field.name img

  current_alignment += get_serialized_size_sensor_msgs__msg__Image(
    &(ros_message->img), current_alignment);
=======
  // field.name transform_init2cur

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->transform_init2cur), current_alignment);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09

  return current_alignment - initial_alignment;
}

static uint32_t _Slam__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface__msg__Slam(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t max_serialized_size_interface__msg__Slam(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

<<<<<<< HEAD
  // member: point_cloud
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        full_bounded, current_alignment);
    }
  }
  // member: cam_pose
=======
  // member: header
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
<<<<<<< HEAD
        max_serialized_size_geometry_msgs__msg__Pose(
        full_bounded, current_alignment);
    }
  }
  // member: world2cam
=======
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: point_cloud
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
<<<<<<< HEAD
        max_serialized_size_geometry_msgs__msg__Pose(
        full_bounded, current_alignment);
    }
  }
  // member: img
=======
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        full_bounded, current_alignment);
    }
  }
  // member: transform_init2cur
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
<<<<<<< HEAD
        max_serialized_size_sensor_msgs__msg__Image(
=======
        max_serialized_size_geometry_msgs__msg__PoseStamped(
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Slam__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_interface__msg__Slam(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Slam = {
  "interface::msg",
  "Slam",
  _Slam__cdr_serialize,
  _Slam__cdr_deserialize,
  _Slam__get_serialized_size,
  _Slam__max_serialized_size
};

static rosidl_message_type_support_t _Slam__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Slam,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, msg, Slam)() {
  return &_Slam__type_support;
}

#if defined(__cplusplus)
}
#endif
