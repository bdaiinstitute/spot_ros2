// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from spot_msgs:srv/SetGripperCameraParameters.idl
// generated code does not contain a copyright notice

#ifndef SPOT_MSGS__SRV__DETAIL__SET_GRIPPER_CAMERA_PARAMETERS__STRUCT_HPP_
#define SPOT_MSGS__SRV__DETAIL__SET_GRIPPER_CAMERA_PARAMETERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'request'
#include "bosdyn_msgs/msg/detail/gripper_camera_param_request__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Request __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Request __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetGripperCameraParameters_Request_
{
  using Type = SetGripperCameraParameters_Request_<ContainerAllocator>;

  explicit SetGripperCameraParameters_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_init)
  {
    (void)_init;
  }

  explicit SetGripperCameraParameters_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _request_type =
    bosdyn_msgs::msg::GripperCameraParamRequest_<ContainerAllocator>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const bosdyn_msgs::msg::GripperCameraParamRequest_<ContainerAllocator> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Request
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Request
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetGripperCameraParameters_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetGripperCameraParameters_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetGripperCameraParameters_Request_

// alias to use template instance with default allocator
using SetGripperCameraParameters_Request =
  spot_msgs::srv::SetGripperCameraParameters_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace spot_msgs


// Include directives for member types
// Member 'response'
#include "bosdyn_msgs/msg/detail/gripper_camera_param_response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Response __attribute__((deprecated))
#else
# define DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Response __declspec(deprecated)
#endif

namespace spot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetGripperCameraParameters_Response_
{
  using Type = SetGripperCameraParameters_Response_<ContainerAllocator>;

  explicit SetGripperCameraParameters_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetGripperCameraParameters_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _response_type =
    bosdyn_msgs::msg::GripperCameraParamResponse_<ContainerAllocator>;
  _response_type response;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__response(
    const bosdyn_msgs::msg::GripperCameraParamResponse_<ContainerAllocator> & _arg)
  {
    this->response = _arg;
    return *this;
  }
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
    spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Response
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__spot_msgs__srv__SetGripperCameraParameters_Response
    std::shared_ptr<spot_msgs::srv::SetGripperCameraParameters_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetGripperCameraParameters_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetGripperCameraParameters_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetGripperCameraParameters_Response_

// alias to use template instance with default allocator
using SetGripperCameraParameters_Response =
  spot_msgs::srv::SetGripperCameraParameters_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace spot_msgs

namespace spot_msgs
{

namespace srv
{

struct SetGripperCameraParameters
{
  using Request = spot_msgs::srv::SetGripperCameraParameters_Request;
  using Response = spot_msgs::srv::SetGripperCameraParameters_Response;
};

}  // namespace srv

}  // namespace spot_msgs

#endif  // SPOT_MSGS__SRV__DETAIL__SET_GRIPPER_CAMERA_PARAMETERS__STRUCT_HPP_
