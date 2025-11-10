// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_state:msg/PoseMultiStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_state/msg/detail/pose_multi_stamped__rosidl_typesupport_introspection_c.h"
#include "robot_state/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_state/msg/detail/pose_multi_stamped__functions.h"
#include "robot_state/msg/detail/pose_multi_stamped__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_state__msg__PoseMultiStamped__init(message_memory);
}

void PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_fini_function(void * message_memory)
{
  robot_state__msg__PoseMultiStamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_member_array[5] = {
  {
    "tick",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__PoseMultiStamped, tick),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "system_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__PoseMultiStamped, system_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ros_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__PoseMultiStamped, ros_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__PoseMultiStamped, source_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__PoseMultiStamped, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_members = {
  "robot_state__msg",  // message namespace
  "PoseMultiStamped",  // message name
  5,  // number of fields
  sizeof(robot_state__msg__PoseMultiStamped),
  PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_member_array,  // message members
  PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_type_support_handle = {
  0,
  &PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_state
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_state, msg, PoseMultiStamped)() {
  if (!PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_type_support_handle.typesupport_identifier) {
    PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PoseMultiStamped__rosidl_typesupport_introspection_c__PoseMultiStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
