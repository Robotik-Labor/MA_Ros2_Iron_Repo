// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from common_services_package:srv/GetPlantpotCoords.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "common_services_package/srv/detail/get_plantpot_coords__functions.h"
#include "common_services_package/srv/detail/get_plantpot_coords__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace common_services_package
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPlantpotCoords_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPlantpotCoords_Request_type_support_ids_t;

static const _GetPlantpotCoords_Request_type_support_ids_t _GetPlantpotCoords_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPlantpotCoords_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPlantpotCoords_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPlantpotCoords_Request_type_support_symbol_names_t _GetPlantpotCoords_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, common_services_package, srv, GetPlantpotCoords_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, common_services_package, srv, GetPlantpotCoords_Request)),
  }
};

typedef struct _GetPlantpotCoords_Request_type_support_data_t
{
  void * data[2];
} _GetPlantpotCoords_Request_type_support_data_t;

static _GetPlantpotCoords_Request_type_support_data_t _GetPlantpotCoords_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPlantpotCoords_Request_message_typesupport_map = {
  2,
  "common_services_package",
  &_GetPlantpotCoords_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetPlantpotCoords_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetPlantpotCoords_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPlantpotCoords_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPlantpotCoords_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &common_services_package__srv__GetPlantpotCoords_Request__get_type_hash,
  &common_services_package__srv__GetPlantpotCoords_Request__get_type_description,
  &common_services_package__srv__GetPlantpotCoords_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace common_services_package

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Request>()
{
  return &::common_services_package::srv::rosidl_typesupport_cpp::GetPlantpotCoords_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, common_services_package, srv, GetPlantpotCoords_Request)() {
  return get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "common_services_package/srv/detail/get_plantpot_coords__functions.h"
// already included above
// #include "common_services_package/srv/detail/get_plantpot_coords__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace common_services_package
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPlantpotCoords_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPlantpotCoords_Response_type_support_ids_t;

static const _GetPlantpotCoords_Response_type_support_ids_t _GetPlantpotCoords_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPlantpotCoords_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPlantpotCoords_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPlantpotCoords_Response_type_support_symbol_names_t _GetPlantpotCoords_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, common_services_package, srv, GetPlantpotCoords_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, common_services_package, srv, GetPlantpotCoords_Response)),
  }
};

typedef struct _GetPlantpotCoords_Response_type_support_data_t
{
  void * data[2];
} _GetPlantpotCoords_Response_type_support_data_t;

static _GetPlantpotCoords_Response_type_support_data_t _GetPlantpotCoords_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPlantpotCoords_Response_message_typesupport_map = {
  2,
  "common_services_package",
  &_GetPlantpotCoords_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetPlantpotCoords_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetPlantpotCoords_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPlantpotCoords_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPlantpotCoords_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &common_services_package__srv__GetPlantpotCoords_Response__get_type_hash,
  &common_services_package__srv__GetPlantpotCoords_Response__get_type_description,
  &common_services_package__srv__GetPlantpotCoords_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace common_services_package

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Response>()
{
  return &::common_services_package::srv::rosidl_typesupport_cpp::GetPlantpotCoords_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, common_services_package, srv, GetPlantpotCoords_Response)() {
  return get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "common_services_package/srv/detail/get_plantpot_coords__functions.h"
// already included above
// #include "common_services_package/srv/detail/get_plantpot_coords__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace common_services_package
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPlantpotCoords_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPlantpotCoords_Event_type_support_ids_t;

static const _GetPlantpotCoords_Event_type_support_ids_t _GetPlantpotCoords_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPlantpotCoords_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPlantpotCoords_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPlantpotCoords_Event_type_support_symbol_names_t _GetPlantpotCoords_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, common_services_package, srv, GetPlantpotCoords_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, common_services_package, srv, GetPlantpotCoords_Event)),
  }
};

typedef struct _GetPlantpotCoords_Event_type_support_data_t
{
  void * data[2];
} _GetPlantpotCoords_Event_type_support_data_t;

static _GetPlantpotCoords_Event_type_support_data_t _GetPlantpotCoords_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPlantpotCoords_Event_message_typesupport_map = {
  2,
  "common_services_package",
  &_GetPlantpotCoords_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetPlantpotCoords_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetPlantpotCoords_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetPlantpotCoords_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPlantpotCoords_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &common_services_package__srv__GetPlantpotCoords_Event__get_type_hash,
  &common_services_package__srv__GetPlantpotCoords_Event__get_type_description,
  &common_services_package__srv__GetPlantpotCoords_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace common_services_package

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Event>()
{
  return &::common_services_package::srv::rosidl_typesupport_cpp::GetPlantpotCoords_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, common_services_package, srv, GetPlantpotCoords_Event)() {
  return get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "common_services_package/srv/detail/get_plantpot_coords__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace common_services_package
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetPlantpotCoords_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetPlantpotCoords_type_support_ids_t;

static const _GetPlantpotCoords_type_support_ids_t _GetPlantpotCoords_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetPlantpotCoords_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetPlantpotCoords_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetPlantpotCoords_type_support_symbol_names_t _GetPlantpotCoords_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, common_services_package, srv, GetPlantpotCoords)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, common_services_package, srv, GetPlantpotCoords)),
  }
};

typedef struct _GetPlantpotCoords_type_support_data_t
{
  void * data[2];
} _GetPlantpotCoords_type_support_data_t;

static _GetPlantpotCoords_type_support_data_t _GetPlantpotCoords_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetPlantpotCoords_service_typesupport_map = {
  2,
  "common_services_package",
  &_GetPlantpotCoords_service_typesupport_ids.typesupport_identifier[0],
  &_GetPlantpotCoords_service_typesupport_symbol_names.symbol_name[0],
  &_GetPlantpotCoords_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetPlantpotCoords_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetPlantpotCoords_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<common_services_package::srv::GetPlantpotCoords_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<common_services_package::srv::GetPlantpotCoords>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<common_services_package::srv::GetPlantpotCoords>,
  &common_services_package__srv__GetPlantpotCoords__get_type_hash,
  &common_services_package__srv__GetPlantpotCoords__get_type_description,
  &common_services_package__srv__GetPlantpotCoords__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace common_services_package

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<common_services_package::srv::GetPlantpotCoords>()
{
  return &::common_services_package::srv::rosidl_typesupport_cpp::GetPlantpotCoords_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
