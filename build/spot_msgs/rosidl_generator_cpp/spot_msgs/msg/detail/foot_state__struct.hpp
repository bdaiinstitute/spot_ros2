// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'foot_position_rt_body'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__FootState __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__FootState __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FootState_
{
  using Type = FootState_<ContainerAllocator>;

  explicit FootState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : foot_position_rt_body(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->contact = 0;
    }
  }

  explicit FootState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : foot_position_rt_body(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->contact = 0;
    }
  }

  // field types and members
  using _foot_position_rt_body_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _foot_position_rt_body_type foot_position_rt_body;
  using _contact_type =
    uint8_t;
  _contact_type contact;

  // setters for named parameter idiom
  Type & set__foot_position_rt_body(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->foot_position_rt_body = _arg;
    return *this;
  }
  Type & set__contact(
    const uint8_t & _arg)
  {
    this->contact = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CONTACT_UNKNOWN =
    0u;
  static constexpr uint8_t CONTACT_MADE =
    1u;
  static constexpr uint8_t CONTACT_LOST =
    2u;

  // pointer types
  using RawPtr =
    spot_msgs::msg::FootState_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::FootState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::FootState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::FootState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::FootState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::FootState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::FootState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::FootState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::FootState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::FootState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__FootState
    std::shared_ptr<spot_msgs::msg::FootState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__FootState
    std::shared_ptr<spot_msgs::msg::FootState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FootState_ & other) const
  {
    if (this->foot_position_rt_body != other.foot_position_rt_body) {
      return false;
    }
    if (this->contact != other.contact) {
      return false;
    }
    return true;
  }
  bool operator!=(const FootState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FootState_

// alias to use template instance with default allocator
using FootState =
  spot_msgs::msg::FootState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FootState_<ContainerAllocator>::CONTACT_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FootState_<ContainerAllocator>::CONTACT_MADE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t FootState_<ContainerAllocator>::CONTACT_LOST;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_
