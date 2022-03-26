// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vesc_msgs:msg/VescImu.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `ypr`
// Member `linear_acceleration`
// Member `angular_velocity`
// Member `compass`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
vesc_msgs__msg__VescImu__init(vesc_msgs__msg__VescImu * msg)
{
  if (!msg) {
    return false;
  }
  // ypr
  if (!geometry_msgs__msg__Vector3__init(&msg->ypr)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // compass
  if (!geometry_msgs__msg__Vector3__init(&msg->compass)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  return true;
}

void
vesc_msgs__msg__VescImu__fini(vesc_msgs__msg__VescImu * msg)
{
  if (!msg) {
    return;
  }
  // ypr
  geometry_msgs__msg__Vector3__fini(&msg->ypr);
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // compass
  geometry_msgs__msg__Vector3__fini(&msg->compass);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
}

vesc_msgs__msg__VescImu *
vesc_msgs__msg__VescImu__create()
{
  vesc_msgs__msg__VescImu * msg = (vesc_msgs__msg__VescImu *)malloc(sizeof(vesc_msgs__msg__VescImu));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vesc_msgs__msg__VescImu));
  bool success = vesc_msgs__msg__VescImu__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
vesc_msgs__msg__VescImu__destroy(vesc_msgs__msg__VescImu * msg)
{
  if (msg) {
    vesc_msgs__msg__VescImu__fini(msg);
  }
  free(msg);
}


bool
vesc_msgs__msg__VescImu__Sequence__init(vesc_msgs__msg__VescImu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  vesc_msgs__msg__VescImu * data = NULL;
  if (size) {
    data = (vesc_msgs__msg__VescImu *)calloc(size, sizeof(vesc_msgs__msg__VescImu));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vesc_msgs__msg__VescImu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vesc_msgs__msg__VescImu__fini(&data[i - 1]);
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
vesc_msgs__msg__VescImu__Sequence__fini(vesc_msgs__msg__VescImu__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vesc_msgs__msg__VescImu__fini(&array->data[i]);
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

vesc_msgs__msg__VescImu__Sequence *
vesc_msgs__msg__VescImu__Sequence__create(size_t size)
{
  vesc_msgs__msg__VescImu__Sequence * array = (vesc_msgs__msg__VescImu__Sequence *)malloc(sizeof(vesc_msgs__msg__VescImu__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = vesc_msgs__msg__VescImu__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
vesc_msgs__msg__VescImu__Sequence__destroy(vesc_msgs__msg__VescImu__Sequence * array)
{
  if (array) {
    vesc_msgs__msg__VescImu__Sequence__fini(array);
  }
  free(array);
}
