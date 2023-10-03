// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/Lease.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__LEASE__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__LEASE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__Lease __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__Lease __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Lease_
{
  using Type = Lease_<ContainerAllocator>;

  explicit Lease_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resource = "";
      this->epoch = "";
    }
  }

  explicit Lease_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : resource(_alloc),
    epoch(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resource = "";
      this->epoch = "";
    }
  }

  // field types and members
  using _resource_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _resource_type resource;
  using _epoch_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _epoch_type epoch;
  using _sequence_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _sequence_type sequence;

  // setters for named parameter idiom
  Type & set__resource(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->resource = _arg;
    return *this;
  }
  Type & set__epoch(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->epoch = _arg;
    return *this;
  }
  Type & set__sequence(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->sequence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::msg::Lease_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::Lease_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::Lease_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::Lease_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::Lease_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::Lease_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::Lease_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::Lease_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::Lease_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::Lease_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__Lease
    std::shared_ptr<spot_msgs::msg::Lease_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__Lease
    std::shared_ptr<spot_msgs::msg::Lease_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Lease_ & other) const
  {
    if (this->resource != other.resource) {
      return false;
    }
    if (this->epoch != other.epoch) {
      return false;
    }
    if (this->sequence != other.sequence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Lease_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Lease_

// alias to use template instance with default allocator
using Lease =
  spot_msgs::msg::Lease_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__LEASE__STRUCT_HPP_
