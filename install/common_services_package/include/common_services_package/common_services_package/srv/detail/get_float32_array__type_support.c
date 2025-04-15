// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from common_services_package:srv/GetFloat32Array.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "common_services_package/srv/detail/get_float32_array__rosidl_typesupport_introspection_c.h"
#include "common_services_package/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "common_services_package/srv/detail/get_float32_array__functions.h"
#include "common_services_package/srv/detail/get_float32_array__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  common_services_package__srv__GetFloat32Array_Request__init(message_memory);
}

void common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_fini_function(void * message_memory)
{
  common_services_package__srv__GetFloat32Array_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_members = {
  "common_services_package__srv",  // message namespace
  "GetFloat32Array_Request",  // message name
  1,  // number of fields
  sizeof(common_services_package__srv__GetFloat32Array_Request),
  common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_member_array,  // message members
  common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_type_support_handle = {
  0,
  &common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_members,
  get_message_typesupport_handle_function,
  &common_services_package__srv__GetFloat32Array_Request__get_type_hash,
  &common_services_package__srv__GetFloat32Array_Request__get_type_description,
  &common_services_package__srv__GetFloat32Array_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_common_services_package
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Request)() {
  if (!common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_type_support_handle.typesupport_identifier) {
    common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "common_services_package/srv/detail/get_float32_array__rosidl_typesupport_introspection_c.h"
// already included above
// #include "common_services_package/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "common_services_package/srv/detail/get_float32_array__functions.h"
// already included above
// #include "common_services_package/srv/detail/get_float32_array__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "std_msgs/msg/float32_multi_array.h"
// Member `data`
#include "std_msgs/msg/detail/float32_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  common_services_package__srv__GetFloat32Array_Response__init(message_memory);
}

void common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_fini_function(void * message_memory)
{
  common_services_package__srv__GetFloat32Array_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Response, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_members = {
  "common_services_package__srv",  // message namespace
  "GetFloat32Array_Response",  // message name
  3,  // number of fields
  sizeof(common_services_package__srv__GetFloat32Array_Response),
  common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_member_array,  // message members
  common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle = {
  0,
  &common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_members,
  get_message_typesupport_handle_function,
  &common_services_package__srv__GetFloat32Array_Response__get_type_hash,
  &common_services_package__srv__GetFloat32Array_Response__get_type_description,
  &common_services_package__srv__GetFloat32Array_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_common_services_package
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Response)() {
  common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32MultiArray)();
  if (!common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle.typesupport_identifier) {
    common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "common_services_package/srv/detail/get_float32_array__rosidl_typesupport_introspection_c.h"
// already included above
// #include "common_services_package/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "common_services_package/srv/detail/get_float32_array__functions.h"
// already included above
// #include "common_services_package/srv/detail/get_float32_array__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "common_services_package/srv/get_float32_array.h"
// Member `request`
// Member `response`
// already included above
// #include "common_services_package/srv/detail/get_float32_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  common_services_package__srv__GetFloat32Array_Event__init(message_memory);
}

void common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_fini_function(void * message_memory)
{
  common_services_package__srv__GetFloat32Array_Event__fini(message_memory);
}

size_t common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__size_function__GetFloat32Array_Event__request(
  const void * untyped_member)
{
  const common_services_package__srv__GetFloat32Array_Request__Sequence * member =
    (const common_services_package__srv__GetFloat32Array_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_const_function__GetFloat32Array_Event__request(
  const void * untyped_member, size_t index)
{
  const common_services_package__srv__GetFloat32Array_Request__Sequence * member =
    (const common_services_package__srv__GetFloat32Array_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_function__GetFloat32Array_Event__request(
  void * untyped_member, size_t index)
{
  common_services_package__srv__GetFloat32Array_Request__Sequence * member =
    (common_services_package__srv__GetFloat32Array_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__fetch_function__GetFloat32Array_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const common_services_package__srv__GetFloat32Array_Request * item =
    ((const common_services_package__srv__GetFloat32Array_Request *)
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_const_function__GetFloat32Array_Event__request(untyped_member, index));
  common_services_package__srv__GetFloat32Array_Request * value =
    (common_services_package__srv__GetFloat32Array_Request *)(untyped_value);
  *value = *item;
}

void common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__assign_function__GetFloat32Array_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  common_services_package__srv__GetFloat32Array_Request * item =
    ((common_services_package__srv__GetFloat32Array_Request *)
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_function__GetFloat32Array_Event__request(untyped_member, index));
  const common_services_package__srv__GetFloat32Array_Request * value =
    (const common_services_package__srv__GetFloat32Array_Request *)(untyped_value);
  *item = *value;
}

bool common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__resize_function__GetFloat32Array_Event__request(
  void * untyped_member, size_t size)
{
  common_services_package__srv__GetFloat32Array_Request__Sequence * member =
    (common_services_package__srv__GetFloat32Array_Request__Sequence *)(untyped_member);
  common_services_package__srv__GetFloat32Array_Request__Sequence__fini(member);
  return common_services_package__srv__GetFloat32Array_Request__Sequence__init(member, size);
}

size_t common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__size_function__GetFloat32Array_Event__response(
  const void * untyped_member)
{
  const common_services_package__srv__GetFloat32Array_Response__Sequence * member =
    (const common_services_package__srv__GetFloat32Array_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_const_function__GetFloat32Array_Event__response(
  const void * untyped_member, size_t index)
{
  const common_services_package__srv__GetFloat32Array_Response__Sequence * member =
    (const common_services_package__srv__GetFloat32Array_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_function__GetFloat32Array_Event__response(
  void * untyped_member, size_t index)
{
  common_services_package__srv__GetFloat32Array_Response__Sequence * member =
    (common_services_package__srv__GetFloat32Array_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__fetch_function__GetFloat32Array_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const common_services_package__srv__GetFloat32Array_Response * item =
    ((const common_services_package__srv__GetFloat32Array_Response *)
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_const_function__GetFloat32Array_Event__response(untyped_member, index));
  common_services_package__srv__GetFloat32Array_Response * value =
    (common_services_package__srv__GetFloat32Array_Response *)(untyped_value);
  *value = *item;
}

void common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__assign_function__GetFloat32Array_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  common_services_package__srv__GetFloat32Array_Response * item =
    ((common_services_package__srv__GetFloat32Array_Response *)
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_function__GetFloat32Array_Event__response(untyped_member, index));
  const common_services_package__srv__GetFloat32Array_Response * value =
    (const common_services_package__srv__GetFloat32Array_Response *)(untyped_value);
  *item = *value;
}

bool common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__resize_function__GetFloat32Array_Event__response(
  void * untyped_member, size_t size)
{
  common_services_package__srv__GetFloat32Array_Response__Sequence * member =
    (common_services_package__srv__GetFloat32Array_Response__Sequence *)(untyped_member);
  common_services_package__srv__GetFloat32Array_Response__Sequence__fini(member);
  return common_services_package__srv__GetFloat32Array_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Event, request),  // bytes offset in struct
    NULL,  // default value
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__size_function__GetFloat32Array_Event__request,  // size() function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_const_function__GetFloat32Array_Event__request,  // get_const(index) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_function__GetFloat32Array_Event__request,  // get(index) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__fetch_function__GetFloat32Array_Event__request,  // fetch(index, &value) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__assign_function__GetFloat32Array_Event__request,  // assign(index, value) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__resize_function__GetFloat32Array_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(common_services_package__srv__GetFloat32Array_Event, response),  // bytes offset in struct
    NULL,  // default value
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__size_function__GetFloat32Array_Event__response,  // size() function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_const_function__GetFloat32Array_Event__response,  // get_const(index) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__get_function__GetFloat32Array_Event__response,  // get(index) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__fetch_function__GetFloat32Array_Event__response,  // fetch(index, &value) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__assign_function__GetFloat32Array_Event__response,  // assign(index, value) function pointer
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__resize_function__GetFloat32Array_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_members = {
  "common_services_package__srv",  // message namespace
  "GetFloat32Array_Event",  // message name
  3,  // number of fields
  sizeof(common_services_package__srv__GetFloat32Array_Event),
  common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_member_array,  // message members
  common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_type_support_handle = {
  0,
  &common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_members,
  get_message_typesupport_handle_function,
  &common_services_package__srv__GetFloat32Array_Event__get_type_hash,
  &common_services_package__srv__GetFloat32Array_Event__get_type_description,
  &common_services_package__srv__GetFloat32Array_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_common_services_package
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Event)() {
  common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Request)();
  common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Response)();
  if (!common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_type_support_handle.typesupport_identifier) {
    common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "common_services_package/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "common_services_package/srv/detail/get_float32_array__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_members = {
  "common_services_package__srv",  // service namespace
  "GetFloat32Array",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_type_support_handle,
  NULL,  // response message
  // common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle
  NULL  // event_message
  // common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle
};


static rosidl_service_type_support_t common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_type_support_handle = {
  0,
  &common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_members,
  get_service_typesupport_handle_function,
  &common_services_package__srv__GetFloat32Array_Request__rosidl_typesupport_introspection_c__GetFloat32Array_Request_message_type_support_handle,
  &common_services_package__srv__GetFloat32Array_Response__rosidl_typesupport_introspection_c__GetFloat32Array_Response_message_type_support_handle,
  &common_services_package__srv__GetFloat32Array_Event__rosidl_typesupport_introspection_c__GetFloat32Array_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    common_services_package,
    srv,
    GetFloat32Array
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    common_services_package,
    srv,
    GetFloat32Array
  ),
  &common_services_package__srv__GetFloat32Array__get_type_hash,
  &common_services_package__srv__GetFloat32Array__get_type_description,
  &common_services_package__srv__GetFloat32Array__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_common_services_package
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array)() {
  if (!common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_type_support_handle.typesupport_identifier) {
    common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common_services_package, srv, GetFloat32Array_Event)()->data;
  }

  return &common_services_package__srv__detail__get_float32_array__rosidl_typesupport_introspection_c__GetFloat32Array_service_type_support_handle;
}
