// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_state:msg/EEFTraj.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__STRUCT_HPP_
#define ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'traj'
#include "robot_state/msg/detail/eef_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_state__msg__EEFTraj __attribute__((deprecated))
#else
# define DEPRECATED__robot_state__msg__EEFTraj __declspec(deprecated)
#endif

namespace robot_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EEFTraj_
{
  using Type = EEFTraj_<ContainerAllocator>;

  explicit EEFTraj_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit EEFTraj_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _traj_type =
    std::vector<robot_state::msg::EEFState_<ContainerAllocator>, typename ContainerAllocator::template rebind<robot_state::msg::EEFState_<ContainerAllocator>>::other>;
  _traj_type traj;

  // setters for named parameter idiom
  Type & set__traj(
    const std::vector<robot_state::msg::EEFState_<ContainerAllocator>, typename ContainerAllocator::template rebind<robot_state::msg::EEFState_<ContainerAllocator>>::other> & _arg)
  {
    this->traj = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_state::msg::EEFTraj_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_state::msg::EEFTraj_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_state::msg::EEFTraj_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_state::msg::EEFTraj_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_state::msg::EEFTraj_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_state::msg::EEFTraj_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_state::msg::EEFTraj_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_state::msg::EEFTraj_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_state::msg::EEFTraj_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_state::msg::EEFTraj_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_state__msg__EEFTraj
    std::shared_ptr<robot_state::msg::EEFTraj_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_state__msg__EEFTraj
    std::shared_ptr<robot_state::msg::EEFTraj_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EEFTraj_ & other) const
  {
    if (this->traj != other.traj) {
      return false;
    }
    return true;
  }
  bool operator!=(const EEFTraj_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EEFTraj_

// alias to use template instance with default allocator
using EEFTraj =
  robot_state::msg::EEFTraj_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_state

#endif  // ROBOT_STATE__MSG__DETAIL__EEF_TRAJ__STRUCT_HPP_
