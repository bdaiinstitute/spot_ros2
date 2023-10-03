// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/LeaseOwner.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__LeaseOwner __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__LeaseOwner __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LeaseOwner_
{
  using Type = LeaseOwner_<ContainerAllocator>;

  explicit LeaseOwner_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->client_name = "";
      this->user_name = "";
    }
  }

  explicit LeaseOwner_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : client_name(_alloc),
    user_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->client_name = "";
      this->user_name = "";
    }
  }

  // field types and members
  using _client_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_name_type client_name;
  using _user_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _user_name_type user_name;

  // setters for named parameter idiom
  Type & set__client_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->client_name = _arg;
    return *this;
  }
  Type & set__user_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->user_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::msg::LeaseOwner_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::LeaseOwner_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::LeaseOwner_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::LeaseOwner_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__LeaseOwner
    std::shared_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__LeaseOwner
    std::shared_ptr<spot_msgs::msg::LeaseOwner_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LeaseOwner_ & other) const
  {
    if (this->client_name != other.client_name) {
      return false;
    }
    if (this->user_name != other.user_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const LeaseOwner_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LeaseOwner_

// alias to use template instance with default allocator
using LeaseOwner =
  spot_msgs::msg::LeaseOwner_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE_OWNER__STRUCT_HPP_
