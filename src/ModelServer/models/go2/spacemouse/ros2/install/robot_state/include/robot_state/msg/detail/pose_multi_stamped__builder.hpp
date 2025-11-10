// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_state:msg/PoseMultiStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__BUILDER_HPP_
#define ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__BUILDER_HPP_

#include "robot_state/msg/detail/pose_multi_stamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_state
{

namespace msg
{

namespace builder
{

class Init_PoseMultiStamped_pose
{
public:
  explicit Init_PoseMultiStamped_pose(::robot_state::msg::PoseMultiStamped & msg)
  : msg_(msg)
  {}
  ::robot_state::msg::PoseMultiStamped pose(::robot_state::msg::PoseMultiStamped::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_state::msg::PoseMultiStamped msg_;
};

class Init_PoseMultiStamped_source_time
{
public:
  explicit Init_PoseMultiStamped_source_time(::robot_state::msg::PoseMultiStamped & msg)
  : msg_(msg)
  {}
  Init_PoseMultiStamped_pose source_time(::robot_state::msg::PoseMultiStamped::_source_time_type arg)
  {
    msg_.source_time = std::move(arg);
    return Init_PoseMultiStamped_pose(msg_);
  }

private:
  ::robot_state::msg::PoseMultiStamped msg_;
};

class Init_PoseMultiStamped_ros_time
{
public:
  explicit Init_PoseMultiStamped_ros_time(::robot_state::msg::PoseMultiStamped & msg)
  : msg_(msg)
  {}
  Init_PoseMultiStamped_source_time ros_time(::robot_state::msg::PoseMultiStamped::_ros_time_type arg)
  {
    msg_.ros_time = std::move(arg);
    return Init_PoseMultiStamped_source_time(msg_);
  }

private:
  ::robot_state::msg::PoseMultiStamped msg_;
};

class Init_PoseMultiStamped_system_time
{
public:
  explicit Init_PoseMultiStamped_system_time(::robot_state::msg::PoseMultiStamped & msg)
  : msg_(msg)
  {}
  Init_PoseMultiStamped_ros_time system_time(::robot_state::msg::PoseMultiStamped::_system_time_type arg)
  {
    msg_.system_time = std::move(arg);
    return Init_PoseMultiStamped_ros_time(msg_);
  }

private:
  ::robot_state::msg::PoseMultiStamped msg_;
};

class Init_PoseMultiStamped_tick
{
public:
  Init_PoseMultiStamped_tick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseMultiStamped_system_time tick(::robot_state::msg::PoseMultiStamped::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_PoseMultiStamped_system_time(msg_);
  }

private:
  ::robot_state::msg::PoseMultiStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_state::msg::PoseMultiStamped>()
{
  return robot_state::msg::builder::Init_PoseMultiStamped_tick();
}

}  // namespace robot_state

#endif  // ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__BUILDER_HPP_
