// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
vesc_msgs__msg__VescState__init(vesc_msgs__msg__VescState * msg)
{
  if (!msg) {
    return false;
  }
  // temp_fet
  // temp_motor
  // current_motor
  // current_input
  // avg_id
  // avg_iq
  // duty_cycle
  // speed
  // voltage_input
  // charge_drawn
  // charge_regen
  // energy_drawn
  // energy_regen
  // displacement
  // distance_traveled
  // fault_code
  // pid_pos_now
  // controller_id
  // ntc_temp_mos1
  // ntc_temp_mos2
  // ntc_temp_mos3
  // avg_vd
  // avg_vq
  return true;
}

void
vesc_msgs__msg__VescState__fini(vesc_msgs__msg__VescState * msg)
{
  if (!msg) {
    return;
  }
  // temp_fet
  // temp_motor
  // current_motor
  // current_input
  // avg_id
  // avg_iq
  // duty_cycle
  // speed
  // voltage_input
  // charge_drawn
  // charge_regen
  // energy_drawn
  // energy_regen
  // displacement
  // distance_traveled
  // fault_code
  // pid_pos_now
  // controller_id
  // ntc_temp_mos1
  // ntc_temp_mos2
  // ntc_temp_mos3
  // avg_vd
  // avg_vq
}

vesc_msgs__msg__VescState *
vesc_msgs__msg__VescState__create()
{
  vesc_msgs__msg__VescState * msg = (vesc_msgs__msg__VescState *)malloc(sizeof(vesc_msgs__msg__VescState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vesc_msgs__msg__VescState));
  bool success = vesc_msgs__msg__VescState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
vesc_msgs__msg__VescState__destroy(vesc_msgs__msg__VescState * msg)
{
  if (msg) {
    vesc_msgs__msg__VescState__fini(msg);
  }
  free(msg);
}


bool
vesc_msgs__msg__VescState__Sequence__init(vesc_msgs__msg__VescState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  vesc_msgs__msg__VescState * data = NULL;
  if (size) {
    data = (vesc_msgs__msg__VescState *)calloc(size, sizeof(vesc_msgs__msg__VescState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vesc_msgs__msg__VescState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vesc_msgs__msg__VescState__fini(&data[i - 1]);
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
vesc_msgs__msg__VescState__Sequence__fini(vesc_msgs__msg__VescState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vesc_msgs__msg__VescState__fini(&array->data[i]);
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

vesc_msgs__msg__VescState__Sequence *
vesc_msgs__msg__VescState__Sequence__create(size_t size)
{
  vesc_msgs__msg__VescState__Sequence * array = (vesc_msgs__msg__VescState__Sequence *)malloc(sizeof(vesc_msgs__msg__VescState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = vesc_msgs__msg__VescState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
vesc_msgs__msg__VescState__Sequence__destroy(vesc_msgs__msg__VescState__Sequence * array)
{
  if (array) {
    vesc_msgs__msg__VescState__Sequence__fini(array);
  }
  free(array);
}
