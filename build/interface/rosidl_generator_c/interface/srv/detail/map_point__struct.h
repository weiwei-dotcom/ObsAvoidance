// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__MAP_POINT__STRUCT_H_
#define INTERFACE__SRV__DETAIL__MAP_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/MapPoint in the package interface.
typedef struct interface__srv__MapPoint_Request
{
  uint8_t structure_needs_at_least_one_member;
} interface__srv__MapPoint_Request;

// Struct for a sequence of interface__srv__MapPoint_Request.
typedef struct interface__srv__MapPoint_Request__Sequence
{
  interface__srv__MapPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__MapPoint_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'cam_pose'
// Member 'world2cam'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in srv/MapPoint in the package interface.
typedef struct interface__srv__MapPoint_Response
{
  sensor_msgs__msg__PointCloud2 point_cloud;
  sensor_msgs__msg__Image img;
  geometry_msgs__msg__Pose cam_pose;
  geometry_msgs__msg__Pose world2cam;
} interface__srv__MapPoint_Response;

// Struct for a sequence of interface__srv__MapPoint_Response.
typedef struct interface__srv__MapPoint_Response__Sequence
{
  interface__srv__MapPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__MapPoint_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__MAP_POINT__STRUCT_H_
