// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice
#include "interface/msg/detail/slam__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
<<<<<<< HEAD
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"
// Member `cam_pose`
// Member `world2cam`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `img`
#include "sensor_msgs/msg/detail/image__functions.h"
=======
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"
// Member `transform_init2cur`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09

bool
interface__msg__Slam__init(interface__msg__Slam * msg)
{
  if (!msg) {
    return false;
  }
<<<<<<< HEAD
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->point_cloud)) {
    interface__msg__Slam__fini(msg);
    return false;
  }
  // cam_pose
  if (!geometry_msgs__msg__Pose__init(&msg->cam_pose)) {
    interface__msg__Slam__fini(msg);
    return false;
  }
  // world2cam
  if (!geometry_msgs__msg__Pose__init(&msg->world2cam)) {
    interface__msg__Slam__fini(msg);
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__init(&msg->img)) {
=======
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    interface__msg__Slam__fini(msg);
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->point_cloud)) {
    interface__msg__Slam__fini(msg);
    return false;
  }
  // transform_init2cur
  if (!geometry_msgs__msg__PoseStamped__init(&msg->transform_init2cur)) {
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    interface__msg__Slam__fini(msg);
    return false;
  }
  return true;
}

void
interface__msg__Slam__fini(interface__msg__Slam * msg)
{
  if (!msg) {
    return;
  }
<<<<<<< HEAD
  // point_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->point_cloud);
  // cam_pose
  geometry_msgs__msg__Pose__fini(&msg->cam_pose);
  // world2cam
  geometry_msgs__msg__Pose__fini(&msg->world2cam);
  // img
  sensor_msgs__msg__Image__fini(&msg->img);
=======
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // point_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->point_cloud);
  // transform_init2cur
  geometry_msgs__msg__PoseStamped__fini(&msg->transform_init2cur);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
}

bool
interface__msg__Slam__are_equal(const interface__msg__Slam * lhs, const interface__msg__Slam * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
<<<<<<< HEAD
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->point_cloud), &(rhs->point_cloud)))
  {
    return false;
  }
  // cam_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->cam_pose), &(rhs->cam_pose)))
  {
    return false;
  }
  // world2cam
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->world2cam), &(rhs->world2cam)))
  {
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->img), &(rhs->img)))
=======
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->point_cloud), &(rhs->point_cloud)))
  {
    return false;
  }
  // transform_init2cur
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->transform_init2cur), &(rhs->transform_init2cur)))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    return false;
  }
  return true;
}

bool
interface__msg__Slam__copy(
  const interface__msg__Slam * input,
  interface__msg__Slam * output)
{
  if (!input || !output) {
    return false;
  }
<<<<<<< HEAD
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->point_cloud), &(output->point_cloud)))
  {
    return false;
  }
  // cam_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->cam_pose), &(output->cam_pose)))
  {
    return false;
  }
  // world2cam
  if (!geometry_msgs__msg__Pose__copy(
      &(input->world2cam), &(output->world2cam)))
  {
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__copy(
      &(input->img), &(output->img)))
=======
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->point_cloud), &(output->point_cloud)))
  {
    return false;
  }
  // transform_init2cur
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->transform_init2cur), &(output->transform_init2cur)))
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
  {
    return false;
  }
  return true;
}

interface__msg__Slam *
interface__msg__Slam__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__msg__Slam * msg = (interface__msg__Slam *)allocator.allocate(sizeof(interface__msg__Slam), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__msg__Slam));
  bool success = interface__msg__Slam__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__msg__Slam__destroy(interface__msg__Slam * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__msg__Slam__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__msg__Slam__Sequence__init(interface__msg__Slam__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__msg__Slam * data = NULL;

  if (size) {
    data = (interface__msg__Slam *)allocator.zero_allocate(size, sizeof(interface__msg__Slam), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__msg__Slam__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__msg__Slam__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interface__msg__Slam__Sequence__fini(interface__msg__Slam__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interface__msg__Slam__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interface__msg__Slam__Sequence *
interface__msg__Slam__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__msg__Slam__Sequence * array = (interface__msg__Slam__Sequence *)allocator.allocate(sizeof(interface__msg__Slam__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__msg__Slam__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__msg__Slam__Sequence__destroy(interface__msg__Slam__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__msg__Slam__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__msg__Slam__Sequence__are_equal(const interface__msg__Slam__Sequence * lhs, const interface__msg__Slam__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__msg__Slam__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__msg__Slam__Sequence__copy(
  const interface__msg__Slam__Sequence * input,
  interface__msg__Slam__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__msg__Slam);
    interface__msg__Slam * data =
      (interface__msg__Slam *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__msg__Slam__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__msg__Slam__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface__msg__Slam__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
