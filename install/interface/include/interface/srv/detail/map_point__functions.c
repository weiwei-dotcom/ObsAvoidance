// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface:srv/MapPoint.idl
// generated code does not contain a copyright notice
#include "interface/srv/detail/map_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
interface__srv__MapPoint_Request__init(interface__srv__MapPoint_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
interface__srv__MapPoint_Request__fini(interface__srv__MapPoint_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
interface__srv__MapPoint_Request__are_equal(const interface__srv__MapPoint_Request * lhs, const interface__srv__MapPoint_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
interface__srv__MapPoint_Request__copy(
  const interface__srv__MapPoint_Request * input,
  interface__srv__MapPoint_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

interface__srv__MapPoint_Request *
interface__srv__MapPoint_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__MapPoint_Request * msg = (interface__srv__MapPoint_Request *)allocator.allocate(sizeof(interface__srv__MapPoint_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__srv__MapPoint_Request));
  bool success = interface__srv__MapPoint_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__srv__MapPoint_Request__destroy(interface__srv__MapPoint_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__srv__MapPoint_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__srv__MapPoint_Request__Sequence__init(interface__srv__MapPoint_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__MapPoint_Request * data = NULL;

  if (size) {
    data = (interface__srv__MapPoint_Request *)allocator.zero_allocate(size, sizeof(interface__srv__MapPoint_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__srv__MapPoint_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__srv__MapPoint_Request__fini(&data[i - 1]);
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
interface__srv__MapPoint_Request__Sequence__fini(interface__srv__MapPoint_Request__Sequence * array)
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
      interface__srv__MapPoint_Request__fini(&array->data[i]);
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

interface__srv__MapPoint_Request__Sequence *
interface__srv__MapPoint_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__MapPoint_Request__Sequence * array = (interface__srv__MapPoint_Request__Sequence *)allocator.allocate(sizeof(interface__srv__MapPoint_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__srv__MapPoint_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__srv__MapPoint_Request__Sequence__destroy(interface__srv__MapPoint_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__srv__MapPoint_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__srv__MapPoint_Request__Sequence__are_equal(const interface__srv__MapPoint_Request__Sequence * lhs, const interface__srv__MapPoint_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__srv__MapPoint_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__srv__MapPoint_Request__Sequence__copy(
  const interface__srv__MapPoint_Request__Sequence * input,
  interface__srv__MapPoint_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__srv__MapPoint_Request);
    interface__srv__MapPoint_Request * data =
      (interface__srv__MapPoint_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__srv__MapPoint_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__srv__MapPoint_Request__fini(&data[i]);
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
    if (!interface__srv__MapPoint_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `point_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"
// Member `img`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `cam_pose`
// Member `world2cam`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
interface__srv__MapPoint_Response__init(interface__srv__MapPoint_Response * msg)
{
  if (!msg) {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->point_cloud)) {
    interface__srv__MapPoint_Response__fini(msg);
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__init(&msg->img)) {
    interface__srv__MapPoint_Response__fini(msg);
    return false;
  }
  // cam_pose
  if (!geometry_msgs__msg__Pose__init(&msg->cam_pose)) {
    interface__srv__MapPoint_Response__fini(msg);
    return false;
  }
  // world2cam
  if (!geometry_msgs__msg__Pose__init(&msg->world2cam)) {
    interface__srv__MapPoint_Response__fini(msg);
    return false;
  }
  return true;
}

void
interface__srv__MapPoint_Response__fini(interface__srv__MapPoint_Response * msg)
{
  if (!msg) {
    return;
  }
  // point_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->point_cloud);
  // img
  sensor_msgs__msg__Image__fini(&msg->img);
  // cam_pose
  geometry_msgs__msg__Pose__fini(&msg->cam_pose);
  // world2cam
  geometry_msgs__msg__Pose__fini(&msg->world2cam);
}

bool
interface__srv__MapPoint_Response__are_equal(const interface__srv__MapPoint_Response * lhs, const interface__srv__MapPoint_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->point_cloud), &(rhs->point_cloud)))
  {
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->img), &(rhs->img)))
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
  return true;
}

bool
interface__srv__MapPoint_Response__copy(
  const interface__srv__MapPoint_Response * input,
  interface__srv__MapPoint_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // point_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->point_cloud), &(output->point_cloud)))
  {
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__copy(
      &(input->img), &(output->img)))
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
  return true;
}

interface__srv__MapPoint_Response *
interface__srv__MapPoint_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__MapPoint_Response * msg = (interface__srv__MapPoint_Response *)allocator.allocate(sizeof(interface__srv__MapPoint_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__srv__MapPoint_Response));
  bool success = interface__srv__MapPoint_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__srv__MapPoint_Response__destroy(interface__srv__MapPoint_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__srv__MapPoint_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__srv__MapPoint_Response__Sequence__init(interface__srv__MapPoint_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__MapPoint_Response * data = NULL;

  if (size) {
    data = (interface__srv__MapPoint_Response *)allocator.zero_allocate(size, sizeof(interface__srv__MapPoint_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__srv__MapPoint_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__srv__MapPoint_Response__fini(&data[i - 1]);
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
interface__srv__MapPoint_Response__Sequence__fini(interface__srv__MapPoint_Response__Sequence * array)
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
      interface__srv__MapPoint_Response__fini(&array->data[i]);
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

interface__srv__MapPoint_Response__Sequence *
interface__srv__MapPoint_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__MapPoint_Response__Sequence * array = (interface__srv__MapPoint_Response__Sequence *)allocator.allocate(sizeof(interface__srv__MapPoint_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__srv__MapPoint_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__srv__MapPoint_Response__Sequence__destroy(interface__srv__MapPoint_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__srv__MapPoint_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__srv__MapPoint_Response__Sequence__are_equal(const interface__srv__MapPoint_Response__Sequence * lhs, const interface__srv__MapPoint_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__srv__MapPoint_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__srv__MapPoint_Response__Sequence__copy(
  const interface__srv__MapPoint_Response__Sequence * input,
  interface__srv__MapPoint_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__srv__MapPoint_Response);
    interface__srv__MapPoint_Response * data =
      (interface__srv__MapPoint_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__srv__MapPoint_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__srv__MapPoint_Response__fini(&data[i]);
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
    if (!interface__srv__MapPoint_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
