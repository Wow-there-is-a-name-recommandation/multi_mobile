// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"
// Member 'state'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RobotState in the package custom_interfaces.
/**
  * Custom message format
 */
typedef struct custom_interfaces__msg__RobotState
{
  rosidl_runtime_c__String robot_id;
  /// x, y, theta 등의 상태 정보
  rosidl_runtime_c__double__Sequence state;
} custom_interfaces__msg__RobotState;

// Struct for a sequence of custom_interfaces__msg__RobotState.
typedef struct custom_interfaces__msg__RobotState__Sequence
{
  custom_interfaces__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
