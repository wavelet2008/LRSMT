// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice

#ifndef WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__BUILDER_HPP_
#define WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__BUILDER_HPP_

#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace webots_ros2_msgs
{

namespace msg
{

namespace builder
{

class Init_WbCameraRecognitionObject_model
{
public:
  explicit Init_WbCameraRecognitionObject_model(::webots_ros2_msgs::msg::WbCameraRecognitionObject & msg)
  : msg_(msg)
  {}
  ::webots_ros2_msgs::msg::WbCameraRecognitionObject model(::webots_ros2_msgs::msg::WbCameraRecognitionObject::_model_type arg)
  {
    msg_.model = std::move(arg);
    return std::move(msg_);
  }

private:
  ::webots_ros2_msgs::msg::WbCameraRecognitionObject msg_;
};

class Init_WbCameraRecognitionObject_colors
{
public:
  explicit Init_WbCameraRecognitionObject_colors(::webots_ros2_msgs::msg::WbCameraRecognitionObject & msg)
  : msg_(msg)
  {}
  Init_WbCameraRecognitionObject_model colors(::webots_ros2_msgs::msg::WbCameraRecognitionObject::_colors_type arg)
  {
    msg_.colors = std::move(arg);
    return Init_WbCameraRecognitionObject_model(msg_);
  }

private:
  ::webots_ros2_msgs::msg::WbCameraRecognitionObject msg_;
};

class Init_WbCameraRecognitionObject_bbox
{
public:
  explicit Init_WbCameraRecognitionObject_bbox(::webots_ros2_msgs::msg::WbCameraRecognitionObject & msg)
  : msg_(msg)
  {}
  Init_WbCameraRecognitionObject_colors bbox(::webots_ros2_msgs::msg::WbCameraRecognitionObject::_bbox_type arg)
  {
    msg_.bbox = std::move(arg);
    return Init_WbCameraRecognitionObject_colors(msg_);
  }

private:
  ::webots_ros2_msgs::msg::WbCameraRecognitionObject msg_;
};

class Init_WbCameraRecognitionObject_pose
{
public:
  explicit Init_WbCameraRecognitionObject_pose(::webots_ros2_msgs::msg::WbCameraRecognitionObject & msg)
  : msg_(msg)
  {}
  Init_WbCameraRecognitionObject_bbox pose(::webots_ros2_msgs::msg::WbCameraRecognitionObject::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_WbCameraRecognitionObject_bbox(msg_);
  }

private:
  ::webots_ros2_msgs::msg::WbCameraRecognitionObject msg_;
};

class Init_WbCameraRecognitionObject_id
{
public:
  Init_WbCameraRecognitionObject_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WbCameraRecognitionObject_pose id(::webots_ros2_msgs::msg::WbCameraRecognitionObject::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_WbCameraRecognitionObject_pose(msg_);
  }

private:
  ::webots_ros2_msgs::msg::WbCameraRecognitionObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::webots_ros2_msgs::msg::WbCameraRecognitionObject>()
{
  return webots_ros2_msgs::msg::builder::Init_WbCameraRecognitionObject_id();
}

}  // namespace webots_ros2_msgs

#endif  // WEBOTS_ROS2_MSGS__MSG__DETAIL__WB_CAMERA_RECOGNITION_OBJECT__BUILDER_HPP_
