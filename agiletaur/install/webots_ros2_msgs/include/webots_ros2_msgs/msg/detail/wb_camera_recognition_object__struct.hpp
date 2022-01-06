// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice

#ifndef WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__STRUCT_HPP_
#define WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'bbox'
#include "vision_msgs/msg/detail/bounding_box2_d__struct.hpp"
// Member 'colors'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObject __attribute__((deprecated))
#else
# define DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObject __declspec(deprecated)
#endif

namespace webots_ros2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WbCameraRecognitionObject_
{
  using Type = WbCameraRecognitionObject_<ContainerAllocator>;

  explicit WbCameraRecognitionObject_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    bbox(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->model = "";
    }
  }

  explicit WbCameraRecognitionObject_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    bbox(_alloc, _init),
    model(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->model = "";
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pose_type pose;
  using _bbox_type =
    vision_msgs::msg::BoundingBox2D_<ContainerAllocator>;
  _bbox_type bbox;
  using _colors_type =
    std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::ColorRGBA_<ContainerAllocator>>::other>;
  _colors_type colors;
  using _model_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _model_type model;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__bbox(
    const vision_msgs::msg::BoundingBox2D_<ContainerAllocator> & _arg)
  {
    this->bbox = _arg;
    return *this;
  }
  Type & set__colors(
    const std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename ContainerAllocator::template rebind<std_msgs::msg::ColorRGBA_<ContainerAllocator>>::other> & _arg)
  {
    this->colors = _arg;
    return *this;
  }
  Type & set__model(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->model = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator> *;
  using ConstRawPtr =
    const webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObject
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__webots_ros2_msgs__msg__WbCameraRecognitionObject
    std::shared_ptr<webots_ros2_msgs::msg::WbCameraRecognitionObject_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WbCameraRecognitionObject_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->bbox != other.bbox) {
      return false;
    }
    if (this->colors != other.colors) {
      return false;
    }
    if (this->model != other.model) {
      return false;
    }
    return true;
  }
  bool operator!=(const WbCameraRecognitionObject_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WbCameraRecognitionObject_

// alias to use template instance with default allocator
using WbCameraRecognitionObject =
  webots_ros2_msgs::msg::WbCameraRecognitionObject_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace webots_ros2_msgs

#endif  // WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__STRUCT_HPP_
