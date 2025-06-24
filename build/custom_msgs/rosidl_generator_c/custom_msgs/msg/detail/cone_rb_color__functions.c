// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/ConeRBColor.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/cone_rb_color__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_msgs__msg__ConeRBColor__init(custom_msgs__msg__ConeRBColor * msg)
{
  if (!msg) {
    return false;
  }
  // range
  // bearing
  // color
  return true;
}

void
custom_msgs__msg__ConeRBColor__fini(custom_msgs__msg__ConeRBColor * msg)
{
  if (!msg) {
    return;
  }
  // range
  // bearing
  // color
}

bool
custom_msgs__msg__ConeRBColor__are_equal(const custom_msgs__msg__ConeRBColor * lhs, const custom_msgs__msg__ConeRBColor * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // bearing
  if (lhs->bearing != rhs->bearing) {
    return false;
  }
  // color
  if (lhs->color != rhs->color) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__ConeRBColor__copy(
  const custom_msgs__msg__ConeRBColor * input,
  custom_msgs__msg__ConeRBColor * output)
{
  if (!input || !output) {
    return false;
  }
  // range
  output->range = input->range;
  // bearing
  output->bearing = input->bearing;
  // color
  output->color = input->color;
  return true;
}

custom_msgs__msg__ConeRBColor *
custom_msgs__msg__ConeRBColor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__ConeRBColor * msg = (custom_msgs__msg__ConeRBColor *)allocator.allocate(sizeof(custom_msgs__msg__ConeRBColor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__ConeRBColor));
  bool success = custom_msgs__msg__ConeRBColor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__ConeRBColor__destroy(custom_msgs__msg__ConeRBColor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__ConeRBColor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__ConeRBColor__Sequence__init(custom_msgs__msg__ConeRBColor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__ConeRBColor * data = NULL;

  if (size) {
    data = (custom_msgs__msg__ConeRBColor *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__ConeRBColor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__ConeRBColor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__ConeRBColor__fini(&data[i - 1]);
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
custom_msgs__msg__ConeRBColor__Sequence__fini(custom_msgs__msg__ConeRBColor__Sequence * array)
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
      custom_msgs__msg__ConeRBColor__fini(&array->data[i]);
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

custom_msgs__msg__ConeRBColor__Sequence *
custom_msgs__msg__ConeRBColor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__ConeRBColor__Sequence * array = (custom_msgs__msg__ConeRBColor__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__ConeRBColor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__ConeRBColor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__ConeRBColor__Sequence__destroy(custom_msgs__msg__ConeRBColor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__ConeRBColor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__ConeRBColor__Sequence__are_equal(const custom_msgs__msg__ConeRBColor__Sequence * lhs, const custom_msgs__msg__ConeRBColor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__ConeRBColor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__ConeRBColor__Sequence__copy(
  const custom_msgs__msg__ConeRBColor__Sequence * input,
  custom_msgs__msg__ConeRBColor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__ConeRBColor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__ConeRBColor * data =
      (custom_msgs__msg__ConeRBColor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__ConeRBColor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__ConeRBColor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__ConeRBColor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
