// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__Feedback __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__Feedback __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Feedback_
{
  using Type = Feedback_<ContainerAllocator>;

  explicit Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->standing = false;
      this->sitting = false;
      this->moving = false;
      this->serial_number = "";
      this->species = "";
      this->version = "";
      this->nickname = "";
      this->computer_serial_number = "";
    }
  }

  explicit Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : serial_number(_alloc),
    species(_alloc),
    version(_alloc),
    nickname(_alloc),
    computer_serial_number(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->standing = false;
      this->sitting = false;
      this->moving = false;
      this->serial_number = "";
      this->species = "";
      this->version = "";
      this->nickname = "";
      this->computer_serial_number = "";
    }
  }

  // field types and members
  using _standing_type =
    bool;
  _standing_type standing;
  using _sitting_type =
    bool;
  _sitting_type sitting;
  using _moving_type =
    bool;
  _moving_type moving;
  using _serial_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serial_number_type serial_number;
  using _species_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _species_type species;
  using _version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _version_type version;
  using _nickname_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _nickname_type nickname;
  using _computer_serial_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _computer_serial_number_type computer_serial_number;

  // setters for named parameter idiom
  Type & set__standing(
    const bool & _arg)
  {
    this->standing = _arg;
    return *this;
  }
  Type & set__sitting(
    const bool & _arg)
  {
    this->sitting = _arg;
    return *this;
  }
  Type & set__moving(
    const bool & _arg)
  {
    this->moving = _arg;
    return *this;
  }
  Type & set__serial_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serial_number = _arg;
    return *this;
  }
  Type & set__species(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->species = _arg;
    return *this;
  }
  Type & set__version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__nickname(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->nickname = _arg;
    return *this;
  }
  Type & set__computer_serial_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->computer_serial_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::msg::Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__Feedback
    std::shared_ptr<spot_msgs::msg::Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__Feedback
    std::shared_ptr<spot_msgs::msg::Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Feedback_ & other) const
  {
    if (this->standing != other.standing) {
      return false;
    }
    if (this->sitting != other.sitting) {
      return false;
    }
    if (this->moving != other.moving) {
      return false;
    }
    if (this->serial_number != other.serial_number) {
      return false;
    }
    if (this->species != other.species) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->nickname != other.nickname) {
      return false;
    }
    if (this->computer_serial_number != other.computer_serial_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Feedback_

// alias to use template instance with default allocator
using Feedback =
  spot_msgs::msg::Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_HPP_
