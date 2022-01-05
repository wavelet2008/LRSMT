// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from webots_ros2_msgs:msg/WbCameraRecognitionObjects.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_objects__rosidl_typesupport_introspection_c.h"
#include "webots_ros2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_objects__functions.h"
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_objects__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `objects`
#include "webots_ros2_msgs/msg/wb_camera_recognition_object.h"
// Member `objects`
#include "webots_ros2_msgs/msg/detail/wb_camera_recognition_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  webots_ros2_msgs__msg__WbCameraRecognitionObjects__init(message_memory);
}

void WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_fini_function(void * message_memory)
{
  webots_ros2_msgs__msg__WbCameraRecognitionObjects__fini(message_memory);
}

size_t WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__size_function__WbCameraRecognitionObject__objects(
  const void * untyped_member)
{
  const webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * member =
    (const webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *)(untyped_member);
  return member->size;
}

const void * WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__get_const_function__WbCameraRecognitionObject__objects(
  const void * untyped_member, size_t index)
{
  const webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * member =
    (const webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *)(untyped_member);
  return &member->data[index];
}

void * WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__get_function__WbCameraRecognitionObject__objects(
  void * untyped_member, size_t index)
{
  webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * member =
    (webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *)(untyped_member);
  return &member->data[index];
}

bool WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__resize_function__WbCameraRecognitionObject__objects(
  void * untyped_member, size_t size)
{
  webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence * member =
    (webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence *)(untyped_member);
  webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__fini(member);
  return webots_ros2_msgs__msg__WbCameraRecognitionObject__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObjects, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(webots_ros2_msgs__msg__WbCameraRecognitionObjects, objects),  // bytes offset in struct
    NULL,  // default value
    WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__size_function__WbCameraRecognitionObject__objects,  // size() function pointer
    WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__get_const_function__WbCameraRecognitionObject__objects,  // get_const(index) function pointer
    WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__get_function__WbCameraRecognitionObject__objects,  // get(index) function pointer
    WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__resize_function__WbCameraRecognitionObject__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_members = {
  "webots_ros2_msgs__msg",  // message namespace
  "WbCameraRecognitionObjects",  // message name
  2,  // number of fields
  sizeof(webots_ros2_msgs__msg__WbCameraRecognitionObjects),
  WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_member_array,  // message members
  WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_init_function,  // function to initialize message memory (memory has to be allocated)
  WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_type_support_handle = {
  0,
  &WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_webots_ros2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, webots_ros2_msgs, msg, WbCameraRecognitionObjects)() {
  WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, webots_ros2_msgs, msg, WbCameraRecognitionObject)();
  if (!WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_type_support_handle.typesupport_identifier) {
    WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &WbCameraRecognitionObjects__rosidl_typesupport_introspection_c__WbCameraRecognitionObjects_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
