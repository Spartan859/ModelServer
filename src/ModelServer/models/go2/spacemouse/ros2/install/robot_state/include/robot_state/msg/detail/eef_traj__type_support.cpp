// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robot_state:msg/EEFTraj.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_state/msg/detail/eef_traj__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robot_state
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EEFTraj_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robot_state::msg::EEFTraj(_init);
}

void EEFTraj_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robot_state::msg::EEFTraj *>(message_memory);
  typed_message->~EEFTraj();
}

size_t size_function__EEFTraj__traj(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robot_state::msg::EEFState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__EEFTraj__traj(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robot_state::msg::EEFState> *>(untyped_member);
  return &member[index];
}

void * get_function__EEFTraj__traj(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robot_state::msg::EEFState> *>(untyped_member);
  return &member[index];
}

void resize_function__EEFTraj__traj(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robot_state::msg::EEFState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EEFTraj_message_member_array[1] = {
  {
    "traj",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robot_state::msg::EEFState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_state::msg::EEFTraj, traj),  // bytes offset in struct
    nullptr,  // default value
    size_function__EEFTraj__traj,  // size() function pointer
    get_const_function__EEFTraj__traj,  // get_const(index) function pointer
    get_function__EEFTraj__traj,  // get(index) function pointer
    resize_function__EEFTraj__traj  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EEFTraj_message_members = {
  "robot_state::msg",  // message namespace
  "EEFTraj",  // message name
  1,  // number of fields
  sizeof(robot_state::msg::EEFTraj),
  EEFTraj_message_member_array,  // message members
  EEFTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  EEFTraj_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EEFTraj_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EEFTraj_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robot_state


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_state::msg::EEFTraj>()
{
  return &::robot_state::msg::rosidl_typesupport_introspection_cpp::EEFTraj_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_state, msg, EEFTraj)() {
  return &::robot_state::msg::rosidl_typesupport_introspection_cpp::EEFTraj_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
