// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObjects.idl
// generated code does not contain a copyright notice

#ifndef WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECTS__STRUCT_HPP_
#define WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECTS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'objects'
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObjects __attribute__((deprecated))
#else
# define DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObjects __declspec(deprecated)
#endif

namespace webots_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WbCameraRecognitionObjects_
{
  using Type = WbCameraRecognitionObjects_<ContainerAllocator>;

  explicit WbCameraRecognitionObjects_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit WbCameraRecognitionObjects_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _objects_type =
    std::vector<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>, typename ContainerAllocator::template rebind<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>::other>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>, typename ContainerAllocator::template rebind<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>::other> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator> *;
  using ConstRawPtr =
    const webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObjects
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObjects
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObjects_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WbCameraRecognitionObjects_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const WbCameraRecognitionObjects_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WbCameraRecognitionObjects_

// alias to use template instance with default allocator
using WbCameraRecognitionObjects =
  webots_ros2_msgs::msg::WbCameraRecognitionObjects_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace webots_ros2_msgs

#endif  // WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECTS__STRUCT_HPP_
