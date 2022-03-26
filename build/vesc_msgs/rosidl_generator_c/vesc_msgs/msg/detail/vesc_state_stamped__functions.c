// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vesc_msgs:msg/VescStateStamped.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_state_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "vesc_msgs/msg/detail/vesc_state__functions.h"

bool
vesc_msgs__msg__VescStateStamped__init(vesc_msgs__msg__VescStateStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vesc_msgs__msg__VescStateStamped__fini(msg);
    return false;
  }
  // state
  if (!vesc_msgs__msg__VescState__init(&msg->state)) {
    vesc_msgs__msg__VescStateStamped__fini(msg);
    return false;
  }
  return true;
}

void
vesc_msgs__msg__VescStateStamped__fini(vesc_msgs__msg__VescStateStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  vesc_msgs__msg__VescState__fini(&msg->state);
}

vesc_msgs__msg__VescStateStamped *
vesc_msgs__msg__VescStateStamped__create()
{
  vesc_msgs__msg__VescStateStamped * msg = (vesc_msgs__msg__VescStateStamped *)malloc(sizeof(vesc_msgs__msg__VescStateStamped));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vesc_msgs__msg__VescStateStamped));
  bool success = vesc_msgs__msg__VescStateStamped__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
vesc_msgs__msg__VescStateStamped__destroy(vesc_msgs__msg__VescStateStamped * msg)
{
  if (msg) {
    vesc_msgs__msg__VescStateStamped__fini(msg);
  }
  free(msg);
}


bool
vesc_msgs__msg__VescStateStamped__Sequence__init(vesc_msgs__msg__VescStateStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  vesc_msgs__msg__VescStateStamped * data = NULL;
  if (size) {
    data = (vesc_msgs__msg__VescStateStamped *)calloc(size, sizeof(vesc_msgs__msg__VescStateStamped));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vesc_msgs__msg__VescStateStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vesc_msgs__msg__VescStateStamped__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vesc_msgs__msg__VescStateStamped__Sequence__fini(vesc_msgs__msg__VescStateStamped__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vesc_msgs__msg__VescStateStamped__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vesc_msgs__msg__VescStateStamped__Sequence *
vesc_msgs__msg__VescStateStamped__Sequence__create(size_t size)
{
  vesc_msgs__msg__VescStateStamped__Sequence * array = (vesc_msgs__msg__VescStateStamped__Sequence *)malloc(sizeof(vesc_msgs__msg__VescStateStamped__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = vesc_msgs__msg__VescStateStamped__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
vesc_msgs__msg__VescStateStamped__Sequence__destroy(vesc_msgs__msg__VescStateStamped__Sequence * array)
{
  if (array) {
    vesc_msgs__msg__VescStateStamped__Sequence__fini(array);
  }
  free(array);
}
