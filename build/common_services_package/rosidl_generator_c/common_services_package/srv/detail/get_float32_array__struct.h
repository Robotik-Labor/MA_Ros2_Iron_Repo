// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from common_services_package:srv/GetFloat32Array.idl
// generated code does not contain a copyright notice

#ifndef COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_FLOAT32_ARRAY__STRUCT_H_
#define COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_FLOAT32_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetFloat32Array in the package common_services_package.
typedef struct common_services_package__srv__GetFloat32Array_Request
{
  uint8_t structure_needs_at_least_one_member;
} common_services_package__srv__GetFloat32Array_Request;

// Struct for a sequence of common_services_package__srv__GetFloat32Array_Request.
typedef struct common_services_package__srv__GetFloat32Array_Request__Sequence
{
  common_services_package__srv__GetFloat32Array_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} common_services_package__srv__GetFloat32Array_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "std_msgs/msg/detail/float32_multi_array__struct.h"

/// Struct defined in srv/GetFloat32Array in the package common_services_package.
typedef struct common_services_package__srv__GetFloat32Array_Response
{
  bool success;
  rosidl_runtime_c__String message;
  std_msgs__msg__Float32MultiArray data;
} common_services_package__srv__GetFloat32Array_Response;

// Struct for a sequence of common_services_package__srv__GetFloat32Array_Response.
typedef struct common_services_package__srv__GetFloat32Array_Response__Sequence
{
  common_services_package__srv__GetFloat32Array_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} common_services_package__srv__GetFloat32Array_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  common_services_package__srv__GetFloat32Array_Event__request__MAX_SIZE = 1
};
// response
enum
{
  common_services_package__srv__GetFloat32Array_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetFloat32Array in the package common_services_package.
typedef struct common_services_package__srv__GetFloat32Array_Event
{
  service_msgs__msg__ServiceEventInfo info;
  common_services_package__srv__GetFloat32Array_Request__Sequence request;
  common_services_package__srv__GetFloat32Array_Response__Sequence response;
} common_services_package__srv__GetFloat32Array_Event;

// Struct for a sequence of common_services_package__srv__GetFloat32Array_Event.
typedef struct common_services_package__srv__GetFloat32Array_Event__Sequence
{
  common_services_package__srv__GetFloat32Array_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} common_services_package__srv__GetFloat32Array_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_FLOAT32_ARRAY__STRUCT_H_
