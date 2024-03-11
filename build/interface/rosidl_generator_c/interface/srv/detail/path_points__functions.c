// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface:srv/PathPoints.idl
// generated code does not contain a copyright notice
#include "interface/srv/detail/path_points__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `start_position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `start_velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
interface__srv__PathPoints_Request__init(interface__srv__PathPoints_Request * msg)
{
  if (!msg) {
    return false;
  }
  // start_position
  if (!geometry_msgs__msg__Point__init(&msg->start_position)) {
    interface__srv__PathPoints_Request__fini(msg);
    return false;
  }
  // start_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->start_velocity)) {
    interface__srv__PathPoints_Request__fini(msg);
    return false;
  }
  return true;
}

void
interface__srv__PathPoints_Request__fini(interface__srv__PathPoints_Request * msg)
{
  if (!msg) {
    return;
  }
  // start_position
  geometry_msgs__msg__Point__fini(&msg->start_position);
  // start_velocity
  geometry_msgs__msg__Vector3__fini(&msg->start_velocity);
}

bool
interface__srv__PathPoints_Request__are_equal(const interface__srv__PathPoints_Request * lhs, const interface__srv__PathPoints_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start_position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->start_position), &(rhs->start_position)))
  {
    return false;
  }
  // start_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->start_velocity), &(rhs->start_velocity)))
  {
    return false;
  }
  return true;
}

bool
interface__srv__PathPoints_Request__copy(
  const interface__srv__PathPoints_Request * input,
  interface__srv__PathPoints_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // start_position
  if (!geometry_msgs__msg__Point__copy(
      &(input->start_position), &(output->start_position)))
  {
    return false;
  }
  // start_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->start_velocity), &(output->start_velocity)))
  {
    return false;
  }
  return true;
}

interface__srv__PathPoints_Request *
interface__srv__PathPoints_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__PathPoints_Request * msg = (interface__srv__PathPoints_Request *)allocator.allocate(sizeof(interface__srv__PathPoints_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__srv__PathPoints_Request));
  bool success = interface__srv__PathPoints_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__srv__PathPoints_Request__destroy(interface__srv__PathPoints_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__srv__PathPoints_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__srv__PathPoints_Request__Sequence__init(interface__srv__PathPoints_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__PathPoints_Request * data = NULL;

  if (size) {
    data = (interface__srv__PathPoints_Request *)allocator.zero_allocate(size, sizeof(interface__srv__PathPoints_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__srv__PathPoints_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__srv__PathPoints_Request__fini(&data[i - 1]);
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
interface__srv__PathPoints_Request__Sequence__fini(interface__srv__PathPoints_Request__Sequence * array)
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
      interface__srv__PathPoints_Request__fini(&array->data[i]);
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

interface__srv__PathPoints_Request__Sequence *
interface__srv__PathPoints_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__PathPoints_Request__Sequence * array = (interface__srv__PathPoints_Request__Sequence *)allocator.allocate(sizeof(interface__srv__PathPoints_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__srv__PathPoints_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__srv__PathPoints_Request__Sequence__destroy(interface__srv__PathPoints_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__srv__PathPoints_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__srv__PathPoints_Request__Sequence__are_equal(const interface__srv__PathPoints_Request__Sequence * lhs, const interface__srv__PathPoints_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__srv__PathPoints_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__srv__PathPoints_Request__Sequence__copy(
  const interface__srv__PathPoints_Request__Sequence * input,
  interface__srv__PathPoints_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__srv__PathPoints_Request);
    interface__srv__PathPoints_Request * data =
      (interface__srv__PathPoints_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__srv__PathPoints_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__srv__PathPoints_Request__fini(&data[i]);
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
    if (!interface__srv__PathPoints_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `path_points`
// already included above
// #include "geometry_msgs/msg/detail/point__functions.h"

bool
interface__srv__PathPoints_Response__init(interface__srv__PathPoints_Response * msg)
{
  if (!msg) {
    return false;
  }
  // path_points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->path_points, 0)) {
    interface__srv__PathPoints_Response__fini(msg);
    return false;
  }
  // success
  return true;
}

void
interface__srv__PathPoints_Response__fini(interface__srv__PathPoints_Response * msg)
{
  if (!msg) {
    return;
  }
  // path_points
  geometry_msgs__msg__Point__Sequence__fini(&msg->path_points);
  // success
}

bool
interface__srv__PathPoints_Response__are_equal(const interface__srv__PathPoints_Response * lhs, const interface__srv__PathPoints_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path_points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->path_points), &(rhs->path_points)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
interface__srv__PathPoints_Response__copy(
  const interface__srv__PathPoints_Response * input,
  interface__srv__PathPoints_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // path_points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->path_points), &(output->path_points)))
  {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

interface__srv__PathPoints_Response *
interface__srv__PathPoints_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__PathPoints_Response * msg = (interface__srv__PathPoints_Response *)allocator.allocate(sizeof(interface__srv__PathPoints_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface__srv__PathPoints_Response));
  bool success = interface__srv__PathPoints_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface__srv__PathPoints_Response__destroy(interface__srv__PathPoints_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface__srv__PathPoints_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface__srv__PathPoints_Response__Sequence__init(interface__srv__PathPoints_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__PathPoints_Response * data = NULL;

  if (size) {
    data = (interface__srv__PathPoints_Response *)allocator.zero_allocate(size, sizeof(interface__srv__PathPoints_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface__srv__PathPoints_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface__srv__PathPoints_Response__fini(&data[i - 1]);
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
interface__srv__PathPoints_Response__Sequence__fini(interface__srv__PathPoints_Response__Sequence * array)
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
      interface__srv__PathPoints_Response__fini(&array->data[i]);
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

interface__srv__PathPoints_Response__Sequence *
interface__srv__PathPoints_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface__srv__PathPoints_Response__Sequence * array = (interface__srv__PathPoints_Response__Sequence *)allocator.allocate(sizeof(interface__srv__PathPoints_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface__srv__PathPoints_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface__srv__PathPoints_Response__Sequence__destroy(interface__srv__PathPoints_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface__srv__PathPoints_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface__srv__PathPoints_Response__Sequence__are_equal(const interface__srv__PathPoints_Response__Sequence * lhs, const interface__srv__PathPoints_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface__srv__PathPoints_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface__srv__PathPoints_Response__Sequence__copy(
  const interface__srv__PathPoints_Response__Sequence * input,
  interface__srv__PathPoints_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface__srv__PathPoints_Response);
    interface__srv__PathPoints_Response * data =
      (interface__srv__PathPoints_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface__srv__PathPoints_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          interface__srv__PathPoints_Response__fini(&data[i]);
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
    if (!interface__srv__PathPoints_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
