// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_state:msg/EEFState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__EEF_STATE__STRUCT_H_
#define ROBOT_STATE__MSG__DETAIL__EEF_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/EEFState in the package robot_state.
typedef struct robot_state__msg__EEFState
{
  uint32_t tick;
  double system_time;
  double eef_pose[7];
  double gripper_pos;
} robot_state__msg__EEFState;

// Struct for a sequence of robot_state__msg__EEFState.
typedef struct robot_state__msg__EEFState__Sequence
{
  robot_state__msg__EEFState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_state__msg__EEFState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_STATE__MSG__DETAIL__EEF_STATE__STRUCT_H_
