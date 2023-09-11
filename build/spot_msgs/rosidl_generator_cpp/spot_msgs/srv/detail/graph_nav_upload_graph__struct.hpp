// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:srv/GraphNavUploadGraph.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__STRUCT_HPP_
#define SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Request __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Request __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GraphNavUploadGraph_Request_
{
  using Type = GraphNavUploadGraph_Request_<ContainerAllocator>;

  explicit GraphNavUploadGraph_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->upload_filepath = "";
    }
  }

  explicit GraphNavUploadGraph_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : upload_filepath(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->upload_filepath = "";
    }
  }

  // field types and members
  using _upload_filepath_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _upload_filepath_type upload_filepath;

  // setters for named parameter idiom
  Type & set__upload_filepath(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->upload_filepath = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Request
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Request
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphNavUploadGraph_Request_ & other) const
  {
    if (this->upload_filepath != other.upload_filepath) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphNavUploadGraph_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphNavUploadGraph_Request_

// alias to use template instance with default allocator
using GraphNavUploadGraph_Request =
  spot_msgs::srv::GraphNavUploadGraph_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace spot_msgs


#ifndef _WIN32
# define DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Response __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Response __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GraphNavUploadGraph_Response_
{
  using Type = GraphNavUploadGraph_Response_<ContainerAllocator>;

  explicit GraphNavUploadGraph_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GraphNavUploadGraph_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Response
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__srv__GraphNavUploadGraph_Response
    std::shared_ptr<spot_msgs::srv::GraphNavUploadGraph_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphNavUploadGraph_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphNavUploadGraph_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphNavUploadGraph_Response_

// alias to use template instance with default allocator
using GraphNavUploadGraph_Response =
  spot_msgs::srv::GraphNavUploadGraph_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace spot_msgs

namespace spot_msgs
{

namespace srv
{

struct GraphNavUploadGraph
{
  using Request = spot_msgs::srv::GraphNavUploadGraph_Request;
  using Response = spot_msgs::srv::GraphNavUploadGraph_Response;
};

}  // namespace srv

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__GRAPH_NAV_UPLOAD_GRAPH__STRUCT_HPP_
