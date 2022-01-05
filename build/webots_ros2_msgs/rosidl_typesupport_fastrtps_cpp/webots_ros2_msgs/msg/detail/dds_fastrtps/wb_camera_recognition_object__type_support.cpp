// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__rosidl_typesupport_fastrtps_cpp.hpp"
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::PoseStamped &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::PoseStamped &);
size_t get_serialized_size(
  const geometry_msgs::msg::PoseStamped &,
  size_t current_alignment);
size_t
max_serialized_size_PoseStamped(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace vision_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const vision_msgs::msg::BoundingBox2D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  vision_msgs::msg::BoundingBox2D &);
size_t get_serialized_size(
  const vision_msgs::msg::BoundingBox2D &,
  size_t current_alignment);
size_t
max_serialized_size_BoundingBox2D(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace vision_msgs

namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::ColorRGBA &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::ColorRGBA &);
size_t get_serialized_size(
  const std_msgs::msg::ColorRGBA &,
  size_t current_alignment);
size_t
max_serialized_size_ColorRGBA(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace webots_ros2_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_webots_ros2_msgs
cdr_serialize(
  const webots_ros2_msgs::msg::WbCameraRecognitionObject & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: id
  cdr << ros_message.id;
  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pose,
    cdr);
  // Member: bbox
  vision_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bbox,
    cdr);
  // Member: colors
  {
    size_t size = ros_message.colors.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.colors[i],
        cdr);
    }
  }
  // Member: model
  cdr << ros_message.model;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_webots_ros2_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  webots_ros2_msgs::msg::WbCameraRecognitionObject & ros_message)
{
  // Member: id
  cdr >> ros_message.id;

  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pose);

  // Member: bbox
  vision_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bbox);

  // Member: colors
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.colors.resize(size);
    for (size_t i = 0; i < size; i++) {
      std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.colors[i]);
    }
  }

  // Member: model
  cdr >> ros_message.model;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_webots_ros2_msgs
get_serialized_size(
  const webots_ros2_msgs::msg::WbCameraRecognitionObject & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pose

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pose, current_alignment);
  // Member: bbox

  current_alignment +=
    vision_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bbox, current_alignment);
  // Member: colors
  {
    size_t array_size = ros_message.colors.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.colors[index], current_alignment);
    }
  }
  // Member: model
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.model.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_webots_ros2_msgs
max_serialized_size_WbCameraRecognitionObject(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PoseStamped(
        full_bounded, current_alignment);
    }
  }

  // Member: bbox
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        vision_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_BoundingBox2D(
        full_bounded, current_alignment);
    }
  }

  // Member: colors
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_ColorRGBA(
        full_bounded, current_alignment);
    }
  }

  // Member: model
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _WbCameraRecognitionObject__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const webots_ros2_msgs::msg::WbCameraRecognitionObject *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _WbCameraRecognitionObject__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<webots_ros2_msgs::msg::WbCameraRecognitionObject *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _WbCameraRecognitionObject__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const webots_ros2_msgs::msg::WbCameraRecognitionObject *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _WbCameraRecognitionObject__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_WbCameraRecognitionObject(full_bounded, 0);
}

static message_type_support_callbacks_t _WbCameraRecognitionObject__callbacks = {
  "webots_ros2_msgs::msg",
  "WbCameraRecognitionObject",
  _WbCameraRecognitionObject__cdr_serialize,
  _WbCameraRecognitionObject__cdr_deserialize,
  _WbCameraRecognitionObject__get_serialized_size,
  _WbCameraRecognitionObject__max_serialized_size
};

static rosidl_message_type_support_t _WbCameraRecognitionObject__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_WbCameraRecognitionObject__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace webots_ros2_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_webots_ros2_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<webots_ros2_msgs::msg::WbCameraRecognitionObject>()
{
  return &webots_ros2_msgs::msg::typesupport_fastrtps_cpp::_WbCameraRecognitionObject__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, webots_ros2_msgs, msg, WbCameraRecognitionObject)() {
  return &webots_ros2_msgs::msg::typesupport_fastrtps_cpp::_WbCameraRecognitionObject__handle;
}

#ifdef __cplusplus
}
#endif
