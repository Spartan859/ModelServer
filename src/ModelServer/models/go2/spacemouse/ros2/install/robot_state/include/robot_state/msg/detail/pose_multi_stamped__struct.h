// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_state:msg/PoseMultiStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__STRUCT_H_
#define ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/PoseMultiStamped in the package robot_state.
typedef struct robot_state__msg__PoseMultiStamped
{
  uint32_t tick;
  double system_time;
  double ros_time;
  double source_time;
  double pose[7];
} robot_state__msg__PoseMultiStamped;

// Struct for a sequence of robot_state__msg__PoseMultiStamped.
typedef struct robot_state__msg__PoseMultiStamped__Sequence
{
  robot_state__msg__PoseMultiStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_state__msg__PoseMultiStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__STRUCT_H_
