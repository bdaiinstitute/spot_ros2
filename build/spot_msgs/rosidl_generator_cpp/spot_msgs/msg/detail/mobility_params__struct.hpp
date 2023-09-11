// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:msg/MobilityParams.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__STRUCT_HPP_
#define SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'body_control'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__msg__MobilityParams __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__msg__MobilityParams __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MobilityParams_
{
  using Type = MobilityParams_<ContainerAllocator>;

  explicit MobilityParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : body_control(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->locomotion_hint = 0ul;
      this->stair_hint = false;
    }
  }

  explicit MobilityParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : body_control(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->locomotion_hint = 0ul;
      this->stair_hint = false;
    }
  }

  // field types and members
  using _body_control_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _body_control_type body_control;
  using _locomotion_hint_type =
    uint32_t;
  _locomotion_hint_type locomotion_hint;
  using _stair_hint_type =
    bool;
  _stair_hint_type stair_hint;

  // setters for named parameter idiom
  Type & set__body_control(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->body_control = _arg;
    return *this;
  }
  Type & set__locomotion_hint(
    const uint32_t & _arg)
  {
    this->locomotion_hint = _arg;
    return *this;
  }
  Type & set__stair_hint(
    const bool & _arg)
  {
    this->stair_hint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::msg::MobilityParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::msg::MobilityParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::MobilityParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::msg::MobilityParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__msg__MobilityParams
    std::shared_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__msg__MobilityParams
    std::shared_ptr<spot_msgs::msg::MobilityParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MobilityParams_ & other) const
  {
    if (this->body_control != other.body_control) {
      return false;
    }
    if (this->locomotion_hint != other.locomotion_hint) {
      return false;
    }
    if (this->stair_hint != other.stair_hint) {
      return false;
    }
    return true;
  }
  bool operator!=(const MobilityParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MobilityParams_

// alias to use template instance with default allocator
using MobilityParams =
  spot_msgs::msg::MobilityParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace spot_msgs

#endif  // SPOT_MSGS__MSG__DETAIL__MOBILITY_PARAMS__STRUCT_HPP_
