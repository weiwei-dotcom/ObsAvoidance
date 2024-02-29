// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__PATH_POINTS__STRUCT_H_
#define INTERFACE__SRV__DETAIL__PATH_POINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'start_speed'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in srv/PathPoints in the package interface.
typedef struct interface__srv__PathPoints_Request
{
  geometry_msgs__msg__Point start_position;
  geometry_msgs__msg__Vector3 start_speed;
} interface__srv__PathPoints_Request;

// Struct for a sequence of interface__srv__PathPoints_Request.
typedef struct interface__srv__PathPoints_Request__Sequence
{
  interface__srv__PathPoints_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__PathPoints_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'path_points'
// already included above
// #include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in srv/PathPoints in the package interface.
typedef struct interface__srv__PathPoints_Response
{
  geometry_msgs__msg__Point__Sequence path_points;
  bool success;
} interface__srv__PathPoints_Response;

// Struct for a sequence of interface__srv__PathPoints_Response.
typedef struct interface__srv__PathPoints_Response__Sequence
{
  interface__srv__PathPoints_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__PathPoints_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__PATH_POINTS__STRUCT_H_
