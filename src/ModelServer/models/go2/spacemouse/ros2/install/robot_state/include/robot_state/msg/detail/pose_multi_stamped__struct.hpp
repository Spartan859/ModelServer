// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_state:msg/PoseMultiStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__STRUCT_HPP_
#define ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__robot_state__msg__PoseMultiStamped __attribute__((deprecated))
#else
# define DEPRECATED__robot_state__msg__PoseMultiStamped __declspec(deprecated)
#endif

namespace robot_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseMultiStamped_
{
  using Type = PoseMultiStamped_<ContainerAllocator>;

  explicit PoseMultiStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tick = 0ul;
      this->system_time = 0.0;
      this->ros_time = 0.0;
      this->source_time = 0.0;
      std::fill<typename std::array<double, 7>::iterator, double>(this->pose.begin(), this->pose.end(), 0.0);
    }
  }

  explicit PoseMultiStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tick = 0ul;
      this->system_time = 0.0;
      this->ros_time = 0.0;
      this->source_time = 0.0;
      std::fill<typename std::array<double, 7>::iterator, double>(this->pose.begin(), this->pose.end(), 0.0);
    }
  }

  // field types and members
  using _tick_type =
    uint32_t;
  _tick_type tick;
  using _system_time_type =
    double;
  _system_time_type system_time;
  using _ros_time_type =
    double;
  _ros_time_type ros_time;
  using _source_time_type =
    double;
  _source_time_type source_time;
  using _pose_type =
    std::array<double, 7>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__tick(
    const uint32_t & _arg)
  {
    this->tick = _arg;
    return *this;
  }
  Type & set__system_time(
    const double & _arg)
  {
    this->system_time = _arg;
    return *this;
  }
  Type & set__ros_time(
    const double & _arg)
  {
    this->ros_time = _arg;
    return *this;
  }
  Type & set__source_time(
    const double & _arg)
  {
    this->source_time = _arg;
    return *this;
  }
  Type & set__pose(
    const std::array<double, 7> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_state::msg::PoseMultiStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_state::msg::PoseMultiStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_state::msg::PoseMultiStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_state::msg::PoseMultiStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_state__msg__PoseMultiStamped
    std::shared_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_state__msg__PoseMultiStamped
    std::shared_ptr<robot_state::msg::PoseMultiStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseMultiStamped_ & other) const
  {
    if (this->tick != other.tick) {
      return false;
    }
    if (this->system_time != other.system_time) {
      return false;
    }
    if (this->ros_time != other.ros_time) {
      return false;
    }
    if (this->source_time != other.source_time) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseMultiStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseMultiStamped_

// alias to use template instance with default allocator
using PoseMultiStamped =
  robot_state::msg::PoseMultiStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_state

#endif  // ROBOT_STATE__MSG__DETAIL__POSE_MULTI_STAMPED__STRUCT_HPP_
