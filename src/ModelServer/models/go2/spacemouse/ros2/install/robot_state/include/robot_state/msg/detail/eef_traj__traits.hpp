// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_state:msg/EEFTraj.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__TRAITS_HPP_
#define ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__TRAITS_HPP_

#include "robot_state/msg/detail/eef_traj__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_state::msg::EEFTraj>()
{
  return "robot_state::msg::EEFTraj";
}

template<>
inline const char * name<robot_state::msg::EEFTraj>()
{
  return "robot_state/msg/EEFTraj";
}

template<>
struct has_fixed_size<robot_state::msg::EEFTraj>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_state::msg::EEFTraj>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_state::msg::EEFTraj>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__TRAITS_HPP_
