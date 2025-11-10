// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_state:msg/EEFState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__EEF_STATE__BUILDER_HPP_
#define ROBOT_STATE__MSG__DETAIL__EEF_STATE__BUILDER_HPP_

#include "robot_state/msg/detail/eef_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_state
{

namespace msg
{

namespace builder
{

class Init_EEFState_gripper_pos
{
public:
  explicit Init_EEFState_gripper_pos(::robot_state::msg::EEFState & msg)
  : msg_(msg)
  {}
  ::robot_state::msg::EEFState gripper_pos(::robot_state::msg::EEFState::_gripper_pos_type arg)
  {
    msg_.gripper_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state::msg::EEFState msg_;
};

class Init_EEFState_eef_pose
{
public:
  explicit Init_EEFState_eef_pose(::robot_state::msg::EEFState & msg)
  : msg_(msg)
  {}
  Init_EEFState_gripper_pos eef_pose(::robot_state::msg::EEFState::_eef_pose_type arg)
  {
    msg_.eef_pose = std::move(arg);
    return Init_EEFState_gripper_pos(msg_);
  }

private:
  ::robot_state::msg::EEFState msg_;
};

class Init_EEFState_system_time
{
public:
  explicit Init_EEFState_system_time(::robot_state::msg::EEFState & msg)
  : msg_(msg)
  {}
  Init_EEFState_eef_pose system_time(::robot_state::msg::EEFState::_system_time_type arg)
  {
    msg_.system_time = std::move(arg);
    return Init_EEFState_eef_pose(msg_);
  }

private:
  ::robot_state::msg::EEFState msg_;
};

class Init_EEFState_tick
{
public:
  Init_EEFState_tick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EEFState_system_time tick(::robot_state::msg::EEFState::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_EEFState_system_time(msg_);
  }

private:
  ::robot_state::msg::EEFState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state::msg::EEFState>()
{
  return robot_state::msg::builder::Init_EEFState_tick();
}

}  // namespace robot_state

#endif  // ROBOT_STATE__MSG__DETAIL__EEF_STATE__BUILDER_HPP_
