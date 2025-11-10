// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_state:msg/EEFState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_state/msg/detail/eef_state__rosidl_typesupport_introspection_c.h"
#include "robot_state/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_state/msg/detail/eef_state__functions.h"
#include "robot_state/msg/detail/eef_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void EEFState__rosidl_typesupport_introspection_c__EEFState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_state__msg__EEFState__init(message_memory);
}

void EEFState__rosidl_typesupport_introspection_c__EEFState_fini_function(void * message_memory)
{
  robot_state__msg__EEFState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember EEFState__rosidl_typesupport_introspection_c__EEFState_message_member_array[4] = {
  {
    "tick",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__EEFState, tick),  // bytes offset in struct
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
    offsetof(robot_state__msg__EEFState, system_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "eef_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__EEFState, eef_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gripper_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__EEFState, gripper_pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers EEFState__rosidl_typesupport_introspection_c__EEFState_message_members = {
  "robot_state__msg",  // message namespace
  "EEFState",  // message name
  4,  // number of fields
  sizeof(robot_state__msg__EEFState),
  EEFState__rosidl_typesupport_introspection_c__EEFState_message_member_array,  // message members
  EEFState__rosidl_typesupport_introspection_c__EEFState_init_function,  // function to initialize message memory (memory has to be allocated)
  EEFState__rosidl_typesupport_introspection_c__EEFState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t EEFState__rosidl_typesupport_introspection_c__EEFState_message_type_support_handle = {
  0,
  &EEFState__rosidl_typesupport_introspection_c__EEFState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_state
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_state, msg, EEFState)() {
  if (!EEFState__rosidl_typesupport_introspection_c__EEFState_message_type_support_handle.typesupport_identifier) {
    EEFState__rosidl_typesupport_introspection_c__EEFState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &EEFState__rosidl_typesupport_introspection_c__EEFState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
