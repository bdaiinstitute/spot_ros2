// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/BehaviorFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__BehaviorFault __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__BehaviorFault __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BehaviorFault_
{
  using Type = BehaviorFault_<ContainerAllocator>;

  explicit BehaviorFault_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->behavior_fault_id = 0ul;
      this->cause = 0;
      this->status = 0;
    }
  }

  explicit BehaviorFault_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->behavior_fault_id = 0ul;
      this->cause = 0;
      this->status = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _behavior_fault_id_type =
    uint32_t;
  _behavior_fault_id_type behavior_fault_id;
  using _cause_type =
    uint8_t;
  _cause_type cause;
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__behavior_fault_id(
    const uint32_t & _arg)
  {
    this->behavior_fault_id = _arg;
    return *this;
  }
  Type & set__cause(
    const uint8_t & _arg)
  {
    this->cause = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CAUSE_UNKNOWN =
    0u;
  static constexpr uint8_t CAUSE_FALL =
    1u;
  static constexpr uint8_t CAUSE_HARDWARE =
    2u;
  static constexpr uint8_t STATUS_UNKNOWN =
    0u;
  static constexpr uint8_t STATUS_CLEARABLE =
    1u;
  static constexpr uint8_t STATUS_UNCLEARABLE =
    2u;

  // pointer types
  using RawPtr =
    spot_msgs::msg::BehaviorFault_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::BehaviorFault_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::BehaviorFault_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::BehaviorFault_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__BehaviorFault
    std::shared_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__BehaviorFault
    std::shared_ptr<spot_msgs::msg::BehaviorFault_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BehaviorFault_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->behavior_fault_id != other.behavior_fault_id) {
      return false;
    }
    if (this->cause != other.cause) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const BehaviorFault_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BehaviorFault_

// alias to use template instance with default allocator
using BehaviorFault =
  spot_msgs::msg::BehaviorFault_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BehaviorFault_<ContainerAllocator>::CAUSE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BehaviorFault_<ContainerAllocator>::CAUSE_FALL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BehaviorFault_<ContainerAllocator>::CAUSE_HARDWARE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BehaviorFault_<ContainerAllocator>::STATUS_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BehaviorFault_<ContainerAllocator>::STATUS_CLEARABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BehaviorFault_<ContainerAllocator>::STATUS_UNCLEARABLE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__BEHAVIOR_FAULT__STRUCT_HPP_
