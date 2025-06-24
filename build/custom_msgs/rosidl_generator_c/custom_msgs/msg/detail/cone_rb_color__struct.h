// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/ConeRBColor.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CONE_RB_COLOR__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__CONE_RB_COLOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ConeRBColor in the package custom_msgs.
/**
  * This message will be used in a Float32MultiArray message.
 */
typedef struct custom_msgs__msg__ConeRBColor
{
  /// Distance to cone in meters
  float range;
  /// Angle to cone in radians relative to vehicle heading
  float bearing;
  /// Color of the cone (yellow=1, blue=2)
  float color;
} custom_msgs__msg__ConeRBColor;

// Struct for a sequence of custom_msgs__msg__ConeRBColor.
typedef struct custom_msgs__msg__ConeRBColor__Sequence
{
  custom_msgs__msg__ConeRBColor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__ConeRBColor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__CONE_RB_COLOR__STRUCT_H_
