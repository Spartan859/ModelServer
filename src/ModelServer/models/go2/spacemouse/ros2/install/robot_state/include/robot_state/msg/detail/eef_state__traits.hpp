// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_state:msg/EEFState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__EEF_STATE__TRAITS_HPP_
#define ROBOT_STATE__MSG__DETAIL__EEF_STATE__TRAITS_HPP_

#include "robot_state/msg/detail/eef_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_state::msg::EEFState>()
{
  return "robot_state::msg::EEFState";
}

template<>
inline const char * name<robot_state::msg::EEFState>()
{
  return "robot_state/msg/EEFState";
}

template<>
struct has_fixed_size<robot_state::msg::EEFState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_state::msg::EEFState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_state::msg::EEFState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_STATE__MSG__DETAIL__EEF_STATE__TRAITS_HPP_
