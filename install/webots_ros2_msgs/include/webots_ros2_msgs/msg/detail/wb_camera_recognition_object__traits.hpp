// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice

#ifndef WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__TRAITS_HPP_
#define WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__TRAITS_HPP_

#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'bbox'
#include "vision_msgs/msg/detail/bounding_box2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<webots_ros2_msgs::msg::WbCameraRecognitionObject>()
{
  return "webots_ros2_msgs::msg::WbCameraRecognitionObject";
}

template<>
inline const char * name<webots_ros2_msgs::msg::WbCameraRecognitionObject>()
{
  return "webots_ros2_msgs/msg/WbCameraRecognitionObject";
}

template<>
struct has_fixed_size<webots_ros2_msgs::msg::WbCameraRecognitionObject>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<webots_ros2_msgs::msg::WbCameraRecognitionObject>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<webots_ros2_msgs::msg::WbCameraRecognitionObject>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__TRAITS_HPP_
