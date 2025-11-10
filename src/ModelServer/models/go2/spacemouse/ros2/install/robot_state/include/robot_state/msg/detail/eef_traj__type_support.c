// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_state:msg/EEFTraj.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_state/msg/detail/eef_traj__rosidl_typesupport_introspection_c.h"
#include "robot_state/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_state/msg/detail/eef_traj__functions.h"
#include "robot_state/msg/detail/eef_traj__struct.h"


// Include directives for member types
// Member `traj`
#include "robot_state/msg/eef_state.h"
// Member `traj`
#include "robot_state/msg/detail/eef_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_state__msg__EEFTraj__init(message_memory);
}

void EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_fini_function(void * message_memory)
{
  robot_state__msg__EEFTraj__fini(message_memory);
}

size_t EEFTraj__rosidl_typesupport_introspection_c__size_function__EEFState__traj(
  const void * untyped_member)
{
  const robot_state__msg__EEFState__Sequence * member =
    (const robot_state__msg__EEFState__Sequence *)(untyped_member);
  return member->size;
}

const void * EEFTraj__rosidl_typesupport_introspection_c__get_const_function__EEFState__traj(
  const void * untyped_member, size_t index)
{
  const robot_state__msg__EEFState__Sequence * member =
    (const robot_state__msg__EEFState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * EEFTraj__rosidl_typesupport_introspection_c__get_function__EEFState__traj(
  void * untyped_member, size_t index)
{
  robot_state__msg__EEFState__Sequence * member =
    (robot_state__msg__EEFState__Sequence *)(untyped_member);
  return &member->data[index];
}

bool EEFTraj__rosidl_typesupport_introspection_c__resize_function__EEFState__traj(
  void * untyped_member, size_t size)
{
  robot_state__msg__EEFState__Sequence * member =
    (robot_state__msg__EEFState__Sequence *)(untyped_member);
  robot_state__msg__EEFState__Sequence__fini(member);
  return robot_state__msg__EEFState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_member_array[1] = {
  {
    "traj",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state__msg__EEFTraj, traj),  // bytes offset in struct
    NULL,  // default value
    EEFTraj__rosidl_typesupport_introspection_c__size_function__EEFState__traj,  // size() function pointer
    EEFTraj__rosidl_typesupport_introspection_c__get_const_function__EEFState__traj,  // get_const(index) function pointer
    EEFTraj__rosidl_typesupport_introspection_c__get_function__EEFState__traj,  // get(index) function pointer
    EEFTraj__rosidl_typesupport_introspection_c__resize_function__EEFState__traj  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_members = {
  "robot_state__msg",  // message namespace
  "EEFTraj",  // message name
  1,  // number of fields
  sizeof(robot_state__msg__EEFTraj),
  EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_member_array,  // message members
  EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_type_support_handle = {
  0,
  &EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_state
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_state, msg, EEFTraj)() {
  EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_state, msg, EEFState)();
  if (!EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_type_support_handle.typesupport_identifier) {
    EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &EEFTraj__rosidl_typesupport_introspection_c__EEFTraj_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
