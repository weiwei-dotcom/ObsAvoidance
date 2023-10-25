// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/T.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__T__STRUCT_H_
#define INTERFACE__SRV__DETAIL__T__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/T in the package interface.
typedef struct interface__srv__T_Request
{
  int32_t request_flag;
} interface__srv__T_Request;

// Struct for a sequence of interface__srv__T_Request.
typedef struct interface__srv__T_Request__Sequence
{
  interface__srv__T_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__T_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'transform_init_to_world'
// Member 'transform_base_to_world'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in srv/T in the package interface.
typedef struct interface__srv__T_Response
{
  geometry_msgs__msg__Pose transform_init_to_world;
  geometry_msgs__msg__Pose transform_base_to_world;
} interface__srv__T_Response;

// Struct for a sequence of interface__srv__T_Response.
typedef struct interface__srv__T_Response__Sequence
{
  interface__srv__T_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__T_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__T__STRUCT_H_
