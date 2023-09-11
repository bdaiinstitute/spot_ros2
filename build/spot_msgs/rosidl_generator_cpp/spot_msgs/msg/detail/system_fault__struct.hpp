// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/SystemFault.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__STRUCT_HPP_

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
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__SystemFault __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__SystemFault __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SystemFault_
{
  using Type = SystemFault_<ContainerAllocator>;

  explicit SystemFault_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    duration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->code = 0l;
      this->uid = 0ull;
      this->error_message = "";
      this->severity = 0;
    }
  }

  explicit SystemFault_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    name(_alloc),
    duration(_alloc, _init),
    error_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->code = 0l;
      this->uid = 0ull;
      this->error_message = "";
      this->severity = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _duration_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _duration_type duration;
  using _code_type =
    int32_t;
  _code_type code;
  using _uid_type =
    uint64_t;
  _uid_type uid;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _attributes_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _attributes_type attributes;
  using _severity_type =
    uint8_t;
  _severity_type severity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__duration(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__code(
    const int32_t & _arg)
  {
    this->code = _arg;
    return *this;
  }
  Type & set__uid(
    const uint64_t & _arg)
  {
    this->uid = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__attributes(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->attributes = _arg;
    return *this;
  }
  Type & set__severity(
    const uint8_t & _arg)
  {
    this->severity = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SEVERITY_UNKNOWN =
    0u;
  static constexpr uint8_t SEVERITY_INFO =
    1u;
  static constexpr uint8_t SEVERITY_WARN =
    2u;
  static constexpr uint8_t SEVERITY_CRITICAL =
    3u;

  // pointer types
  using RawPtr =
    spot_msgs::msg::SystemFault_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::SystemFault_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::SystemFault_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::SystemFault_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__SystemFault
    std::shared_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__SystemFault
    std::shared_ptr<spot_msgs::msg::SystemFault_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SystemFault_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->code != other.code) {
      return false;
    }
    if (this->uid != other.uid) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->attributes != other.attributes) {
      return false;
    }
    if (this->severity != other.severity) {
      return false;
    }
    return true;
  }
  bool operator!=(const SystemFault_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SystemFault_

// alias to use template instance with default allocator
using SystemFault =
  spot_msgs::msg::SystemFault_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemFault_<ContainerAllocator>::SEVERITY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemFault_<ContainerAllocator>::SEVERITY_INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemFault_<ContainerAllocator>::SEVERITY_WARN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SystemFault_<ContainerAllocator>::SEVERITY_CRITICAL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__SYSTEM_FAULT__STRUCT_HPP_
