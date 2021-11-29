// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObject.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__rosidl_typesupport_introspection_c.h"
#include "webots_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__functions.h"
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `bbox`
#include "vision_msgs/msg/bounding_box2_d.h"
// Member `bbox`
#include "vision_msgs/msg/detail/bounding_box2_d__rosidl_typesupport_introspection_c.h"
// Member `colors`
#include "std_msgs/msg/color_rgba.h"
// Member `colors`
#include "std_msgs/msg/detail/color_rgba__rosidl_typesupport_introspection_c.h"
// Member `model`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  webots_ros2_msgs__msg__WbCameraRecognitionObject__init(message_memory);
}

void WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_fini_function(void * message_memory)
{
  webots_ros2_msgs__msg__WbCameraRecognitionObject__fini(message_memory);
}

size_t WbCameraRecognitionObject__rosidl_typesupport_introspection_c__size_function__ColorRGBA__colors(
  const void * untyped_member)
{
  const std_msgs__msg__ColorRGBA__Sequence * member =
    (const std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  return member->size;
}

const void * WbCameraRecognitionObject__rosidl_typesupport_introspection_c__get_const_function__ColorRGBA__colors(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__ColorRGBA__Sequence * member =
    (const std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  return &member->data[index];
}

void * WbCameraRecognitionObject__rosidl_typesupport_introspection_c__get_function__ColorRGBA__colors(
  void * untyped_member, size_t index)
{
  std_msgs__msg__ColorRGBA__Sequence * member =
    (std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  return &member->data[index];
}

bool WbCameraRecognitionObject__rosidl_typesupport_introspection_c__resize_function__ColorRGBA__colors(
  void * untyped_member, size_t size)
{
  std_msgs__msg__ColorRGBA__Sequence * member =
    (std_msgs__msg__ColorRGBA__Sequence *)(untyped_member);
  std_msgs__msg__ColorRGBA__Sequence__fini(member);
  return std_msgs__msg__ColorRGBA__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_member_array[5] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObject, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObject, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObject, bbox),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "colors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObject, colors),  // bytes offset in struct
    NULL,  // default value
    WbCameraRecognitionObject__rosidl_typesupport_introspection_c__size_function__ColorRGBA__colors,  // size() function pointer
    WbCameraRecognitionObject__rosidl_typesupport_introspection_c__get_const_function__ColorRGBA__colors,  // get_const(index) function pointer
    WbCameraRecognitionObject__rosidl_typesupport_introspection_c__get_function__ColorRGBA__colors,  // get(index) function pointer
    WbCameraRecognitionObject__rosidl_typesupport_introspection_c__resize_function__ColorRGBA__colors  // resize(index) function pointer
  },
  {
    "model",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObject, model),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_members = {
  "webots_ros2_msgs__msg",  // message namespace
  "WbCameraRecognitionObject",  // message name
  5,  // number of fields
  sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObject),
  WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_member_array,  // message members
  WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_init_function,  // function to initialize message memory (memory has to be allocated)
  WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_type_support_handle = {
  0,
  &WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_webots_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, webots_ros2_msgs, msg, WbCameraRecognitionObject)() {
  WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, BoundingBox2D)();
  WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, ColorRGBA)();
  if (!WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_type_support_handle.typesupport_identifier) {
    WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &WbCameraRecognitionObject__rosidl_typesupport_introspection_c__WbCameraRecognitionObject_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
