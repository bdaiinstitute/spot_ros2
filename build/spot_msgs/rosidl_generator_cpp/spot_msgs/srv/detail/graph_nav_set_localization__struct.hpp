// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:srv/GraphNavSetLocalization.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_SET_LOCALIZATION__STRUCT_HPP_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_SET_LOCALIZATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Request __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Request __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GraphNavSetLocalization_Request_
{
  using Type = GraphNavSetLocalization_Request_<ContainerAllocator>;

  explicit GraphNavSetLocalization_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->method = "";
      this->waypoint_id = "";
    }
  }

  explicit GraphNavSetLocalization_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : method(_alloc),
    waypoint_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->method = "";
      this->waypoint_id = "";
    }
  }

  // field types and members
  using _method_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _method_type method;
  using _waypoint_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _waypoint_id_type waypoint_id;

  // setters for named parameter idiom
  Type & set__method(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->method = _arg;
    return *this;
  }
  Type & set__waypoint_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->waypoint_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Request
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Request
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphNavSetLocalization_Request_ & other) const
  {
    if (this->method != other.method) {
      return false;
    }
    if (this->waypoint_id != other.waypoint_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphNavSetLocalization_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphNavSetLocalization_Request_

// alias to use template instance with default allocator
using GraphNavSetLocalization_Request =
  spot_msgs::srv::GraphNavSetLocalization_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace spot_msgs


#ifndef _WIN32
# define DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Response __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Response __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GraphNavSetLocalization_Response_
{
  using Type = GraphNavSetLocalization_Response_<ContainerAllocator>;

  explicit GraphNavSetLocalization_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GraphNavSetLocalization_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Response
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__srv__GraphNavSetLocalization_Response
    std::shared_ptr<spot_msgs::srv::GraphNavSetLocalization_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphNavSetLocalization_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphNavSetLocalization_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphNavSetLocalization_Response_

// alias to use template instance with default allocator
using GraphNavSetLocalization_Response =
  spot_msgs::srv::GraphNavSetLocalization_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace spot_msgs

namespace spot_msgs
{

namespace srv
{

struct GraphNavSetLocalization
{
  using Request = spot_msgs::srv::GraphNavSetLocalization_Request;
  using Response = spot_msgs::srv::GraphNavSetLocalization_Response;
};

}  // namespace srv

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_SET_LOCALIZATION__STRUCT_HPP_
