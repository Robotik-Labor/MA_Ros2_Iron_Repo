// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_services_package:srv/GetPlantpotCoords.idl
// generated code does not contain a copyright notice

#ifndef COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_PLANTPOT_COORDS__STRUCT_HPP_
#define COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_PLANTPOT_COORDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_services_package__srv__GetPlantpotCoords_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_services_package__srv__GetPlantpotCoords_Request __declspec(deprecated)
#endif

namespace common_services_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPlantpotCoords_Request_
{
  using Type = GetPlantpotCoords_Request_<ContainerAllocator>;

  explicit GetPlantpotCoords_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetPlantpotCoords_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_services_package__srv__GetPlantpotCoords_Request
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_services_package__srv__GetPlantpotCoords_Request
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPlantpotCoords_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPlantpotCoords_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPlantpotCoords_Request_

// alias to use template instance with default allocator
using GetPlantpotCoords_Request =
  common_services_package::srv::GetPlantpotCoords_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_services_package


// Include directives for member types
// Member 'coordinates'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_services_package__srv__GetPlantpotCoords_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_services_package__srv__GetPlantpotCoords_Response __declspec(deprecated)
#endif

namespace common_services_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPlantpotCoords_Response_
{
  using Type = GetPlantpotCoords_Response_<ContainerAllocator>;

  explicit GetPlantpotCoords_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coordinates(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GetPlantpotCoords_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    coordinates(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _coordinates_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _coordinates_type coordinates;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__coordinates(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->coordinates = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_services_package__srv__GetPlantpotCoords_Response
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_services_package__srv__GetPlantpotCoords_Response
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPlantpotCoords_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->coordinates != other.coordinates) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPlantpotCoords_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPlantpotCoords_Response_

// alias to use template instance with default allocator
using GetPlantpotCoords_Response =
  common_services_package::srv::GetPlantpotCoords_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_services_package


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common_services_package__srv__GetPlantpotCoords_Event __attribute__((deprecated))
#else
# define DEPRECATED__common_services_package__srv__GetPlantpotCoords_Event __declspec(deprecated)
#endif

namespace common_services_package
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPlantpotCoords_Event_
{
  using Type = GetPlantpotCoords_Event_<ContainerAllocator>;

  explicit GetPlantpotCoords_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetPlantpotCoords_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_services_package::srv::GetPlantpotCoords_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<common_services_package::srv::GetPlantpotCoords_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_services_package__srv__GetPlantpotCoords_Event
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_services_package__srv__GetPlantpotCoords_Event
    std::shared_ptr<common_services_package::srv::GetPlantpotCoords_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPlantpotCoords_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPlantpotCoords_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPlantpotCoords_Event_

// alias to use template instance with default allocator
using GetPlantpotCoords_Event =
  common_services_package::srv::GetPlantpotCoords_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace common_services_package

namespace common_services_package
{

namespace srv
{

struct GetPlantpotCoords
{
  using Request = common_services_package::srv::GetPlantpotCoords_Request;
  using Response = common_services_package::srv::GetPlantpotCoords_Response;
  using Event = common_services_package::srv::GetPlantpotCoords_Event;
};

}  // namespace srv

}  // namespace common_services_package

#endif  // COMMON_SERVICES_PACKAGE__SRV__DETAIL__GET_PLANTPOT_COORDS__STRUCT_HPP_
