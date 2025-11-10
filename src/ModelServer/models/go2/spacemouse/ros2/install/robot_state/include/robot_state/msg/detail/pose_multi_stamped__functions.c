// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_state:msg/PoseMultiStamped.idl
// generated code does not contain a copyright notice
#include "robot_state/msg/detail/pose_multi_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_state__msg__PoseMultiStamped__init(robot_state__msg__PoseMultiStamped * msg)
{
  if (!msg) {
    return false;
  }
  // tick
  // system_time
  // ros_time
  // source_time
  // pose
  return true;
}

void
robot_state__msg__PoseMultiStamped__fini(robot_state__msg__PoseMultiStamped * msg)
{
  if (!msg) {
    return;
  }
  // tick
  // system_time
  // ros_time
  // source_time
  // pose
}

bool
robot_state__msg__PoseMultiStamped__are_equal(const robot_state__msg__PoseMultiStamped * lhs, const robot_state__msg__PoseMultiStamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tick
  if (lhs->tick != rhs->tick) {
    return false;
  }
  // system_time
  if (lhs->system_time != rhs->system_time) {
    return false;
  }
  // ros_time
  if (lhs->ros_time != rhs->ros_time) {
    return false;
  }
  // source_time
  if (lhs->source_time != rhs->source_time) {
    return false;
  }
  // pose
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->pose[i] != rhs->pose[i]) {
      return false;
    }
  }
  return true;
}

bool
robot_state__msg__PoseMultiStamped__copy(
  const robot_state__msg__PoseMultiStamped * input,
  robot_state__msg__PoseMultiStamped * output)
{
  if (!input || !output) {
    return false;
  }
  // tick
  output->tick = input->tick;
  // system_time
  output->system_time = input->system_time;
  // ros_time
  output->ros_time = input->ros_time;
  // source_time
  output->source_time = input->source_time;
  // pose
  for (size_t i = 0; i < 7; ++i) {
    output->pose[i] = input->pose[i];
  }
  return true;
}

robot_state__msg__PoseMultiStamped *
robot_state__msg__PoseMultiStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state__msg__PoseMultiStamped * msg = (robot_state__msg__PoseMultiStamped *)allocator.allocate(sizeof(robot_state__msg__PoseMultiStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_state__msg__PoseMultiStamped));
  bool success = robot_state__msg__PoseMultiStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_state__msg__PoseMultiStamped__destroy(robot_state__msg__PoseMultiStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_state__msg__PoseMultiStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_state__msg__PoseMultiStamped__Sequence__init(robot_state__msg__PoseMultiStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state__msg__PoseMultiStamped * data = NULL;

  if (size) {
    data = (robot_state__msg__PoseMultiStamped *)allocator.zero_allocate(size, sizeof(robot_state__msg__PoseMultiStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_state__msg__PoseMultiStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_state__msg__PoseMultiStamped__fini(&data[i - 1]);
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
robot_state__msg__PoseMultiStamped__Sequence__fini(robot_state__msg__PoseMultiStamped__Sequence * array)
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
      robot_state__msg__PoseMultiStamped__fini(&array->data[i]);
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

robot_state__msg__PoseMultiStamped__Sequence *
robot_state__msg__PoseMultiStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_state__msg__PoseMultiStamped__Sequence * array = (robot_state__msg__PoseMultiStamped__Sequence *)allocator.allocate(sizeof(robot_state__msg__PoseMultiStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_state__msg__PoseMultiStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_state__msg__PoseMultiStamped__Sequence__destroy(robot_state__msg__PoseMultiStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_state__msg__PoseMultiStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_state__msg__PoseMultiStamped__Sequence__are_equal(const robot_state__msg__PoseMultiStamped__Sequence * lhs, const robot_state__msg__PoseMultiStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_state__msg__PoseMultiStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_state__msg__PoseMultiStamped__Sequence__copy(
  const robot_state__msg__PoseMultiStamped__Sequence * input,
  robot_state__msg__PoseMultiStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_state__msg__PoseMultiStamped);
    robot_state__msg__PoseMultiStamped * data =
      (robot_state__msg__PoseMultiStamped *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_state__msg__PoseMultiStamped__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          robot_state__msg__PoseMultiStamped__fini(&data[i]);
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
    if (!robot_state__msg__PoseMultiStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
