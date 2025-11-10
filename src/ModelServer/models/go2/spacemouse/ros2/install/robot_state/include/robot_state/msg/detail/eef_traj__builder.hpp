// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_state:msg/EEFTraj.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__BUILDER_HPP_
#define ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__BUILDER_HPP_

#include "robot_state/msg/detail/eef_traj__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_state
{

namespace msg
{

namespace builder
{

class Init_EEFTraj_traj
{
public:
  Init_EEFTraj_traj()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_state::msg::EEFTraj traj(::robot_state::msg::EEFTraj::_traj_type arg)
  {
    msg_.traj = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state::msg::EEFTraj msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state::msg::EEFTraj>()
{
  return robot_state::msg::builder::Init_EEFTraj_traj();
}

}  // namespace robot_state

#endif  // ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__BUILDER_HPP_
