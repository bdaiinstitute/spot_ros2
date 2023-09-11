// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/Metrics.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__METRICS__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__METRICS__STRUCT_HPP_

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
// Member 'time_moving'
// Member 'electric_power'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__Metrics __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__Metrics __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Metrics_
{
  using Type = Metrics_<ContainerAllocator>;

  explicit Metrics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time_moving(_init),
    electric_power(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0f;
      this->gait_cycles = 0l;
    }
  }

  explicit Metrics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time_moving(_alloc, _init),
    electric_power(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0f;
      this->gait_cycles = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _distance_type =
    float;
  _distance_type distance;
  using _gait_cycles_type =
    int32_t;
  _gait_cycles_type gait_cycles;
  using _time_moving_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_moving_type time_moving;
  using _electric_power_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _electric_power_type electric_power;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__gait_cycles(
    const int32_t & _arg)
  {
    this->gait_cycles = _arg;
    return *this;
  }
  Type & set__time_moving(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_moving = _arg;
    return *this;
  }
  Type & set__electric_power(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->electric_power = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::msg::Metrics_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::Metrics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::Metrics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::Metrics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::Metrics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::Metrics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::Metrics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::Metrics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::Metrics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::Metrics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__Metrics
    std::shared_ptr<spot_msgs::msg::Metrics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__Metrics
    std::shared_ptr<spot_msgs::msg::Metrics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Metrics_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->gait_cycles != other.gait_cycles) {
      return false;
    }
    if (this->time_moving != other.time_moving) {
      return false;
    }
    if (this->electric_power != other.electric_power) {
      return false;
    }
    return true;
  }
  bool operator!=(const Metrics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Metrics_

// alias to use template instance with default allocator
using Metrics =
  spot_msgs::msg::Metrics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__METRICS__STRUCT_HPP_
