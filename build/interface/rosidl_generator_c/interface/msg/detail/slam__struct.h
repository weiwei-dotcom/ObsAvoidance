// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__SLAM__STRUCT_H_
#define INTERFACE__MSG__DETAIL__SLAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'point_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"
// Member 'transform_init2cur'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

// Struct defined in msg/Slam in the package interface.
typedef struct interface__msg__Slam
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__PointCloud2 point_cloud;
  geometry_msgs__msg__PoseStamped transform_init2cur;
} interface__msg__Slam;

// Struct for a sequence of interface__msg__Slam.
typedef struct interface__msg__Slam__Sequence
{
  interface__msg__Slam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__msg__Slam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__MSG__DETAIL__SLAM__STRUCT_H_
