// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common_services_package:srv/GetPointCloud2.idl
// generated code does not contain a copyright notice

#ifndef COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_POINT_CLOUD2__BUILDER_HPP_
#define COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_POINT_CLOUD2__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "common_services_package/srv/detail/get_point_cloud2__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace common_services_package
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_services_package::srv::GetPointCloud2_Request>()
{
  return ::common_services_package::srv::GetPointCloud2_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace common_services_package


namespace common_services_package
{

namespace srv
{

namespace builder
{

class Init_GetPointCloud2_Response_cloud
{
public:
  explicit Init_GetPointCloud2_Response_cloud(::common_services_package::srv::GetPointCloud2_Response & msg)
  : msg_(msg)
  {}
  ::common_services_package::srv::GetPointCloud2_Response cloud(::common_services_package::srv::GetPointCloud2_Response::_cloud_type arg)
  {
    msg_.cloud = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_services_package::srv::GetPointCloud2_Response msg_;
};

class Init_GetPointCloud2_Response_message
{
public:
  explicit Init_GetPointCloud2_Response_message(::common_services_package::srv::GetPointCloud2_Response & msg)
  : msg_(msg)
  {}
  Init_GetPointCloud2_Response_cloud message(::common_services_package::srv::GetPointCloud2_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetPointCloud2_Response_cloud(msg_);
  }

private:
  ::common_services_package::srv::GetPointCloud2_Response msg_;
};

class Init_GetPointCloud2_Response_success
{
public:
  Init_GetPointCloud2_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPointCloud2_Response_message success(::common_services_package::srv::GetPointCloud2_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetPointCloud2_Response_message(msg_);
  }

private:
  ::common_services_package::srv::GetPointCloud2_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_services_package::srv::GetPointCloud2_Response>()
{
  return common_services_package::srv::builder::Init_GetPointCloud2_Response_success();
}

}  // namespace common_services_package


namespace common_services_package
{

namespace srv
{

namespace builder
{

class Init_GetPointCloud2_Event_response
{
public:
  explicit Init_GetPointCloud2_Event_response(::common_services_package::srv::GetPointCloud2_Event & msg)
  : msg_(msg)
  {}
  ::common_services_package::srv::GetPointCloud2_Event response(::common_services_package::srv::GetPointCloud2_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common_services_package::srv::GetPointCloud2_Event msg_;
};

class Init_GetPointCloud2_Event_request
{
public:
  explicit Init_GetPointCloud2_Event_request(::common_services_package::srv::GetPointCloud2_Event & msg)
  : msg_(msg)
  {}
  Init_GetPointCloud2_Event_response request(::common_services_package::srv::GetPointCloud2_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetPointCloud2_Event_response(msg_);
  }

private:
  ::common_services_package::srv::GetPointCloud2_Event msg_;
};

class Init_GetPointCloud2_Event_info
{
public:
  Init_GetPointCloud2_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPointCloud2_Event_request info(::common_services_package::srv::GetPointCloud2_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetPointCloud2_Event_request(msg_);
  }

private:
  ::common_services_package::srv::GetPointCloud2_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::common_services_package::srv::GetPointCloud2_Event>()
{
  return common_services_package::srv::builder::Init_GetPointCloud2_Event_info();
}

}  // namespace common_services_package

#endif  // COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_POINT_CLOUD2__BUILDER_HPP_
