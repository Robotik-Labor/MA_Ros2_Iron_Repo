cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(common_services_package REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)




add_executable(ur5_moveit_test src/ur5_moveit_test.cpp)
add_executable(gripper_close_moveit_test src/gripper_close_moveit_test.cpp)
add_executable(gripper_open_moveit_test src/gripper_open_moveit_test.cpp)
add_executable(ur5_moveit_test_launch src/ur5_moveit_test_launch.cpp)
add_executable(ur5_moveit_test_object_detection src/ur5_moveit_test_object_detection.cpp)
add_executable(ur5_read_positions src/ur5_read_positions.cpp)
add_executable(ur5_pickup_sensor src/ur5_pickup_sensor.cpp)
add_executable(ur5_drop_sensor src/ur5_drop_sensor.cpp)
add_executable(ur5_probe_sensor src/ur5_probe_sensor.cpp)
add_executable(ur5_probe_sensor_service src/ur5_probe_sensor_service.cpp)
add_executable(ur5_move_to_home src/ur5_move_to_home.cpp)
add_executable(ur5_move_to_camera src/ur5_move_to_camera.cpp)
add_executable(ur5_image_octmap_path src/ur5_image_octmap_path.cpp)

target_compile_features(ur5_moveit_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(gripper_close_moveit_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(gripper_open_moveit_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ur5_moveit_test_launch PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ur5_moveit_test_object_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ur5_read_positions PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ur5_pickup_sensor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ur5_pickup_sensor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  ur5_moveit_test
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  ur5_moveit_test_launch
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  ur5_moveit_test_object_detection
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  gripper_open_moveit_test
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  gripper_close_moveit_test
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  ur5_read_positions
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  ur5_pickup_sensor
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
)

ament_target_dependencies(
  ur5_drop_sensor
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
)
ament_target_dependencies(
   ur5_probe_sensor
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
)
ament_target_dependencies(
   ur5_move_to_home
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
)
ament_target_dependencies(
   ur5_move_to_camera
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
)

ament_target_dependencies(
   ur5_probe_sensor_service
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
  "common_services_package"
)

ament_target_dependencies(
   ur5_image_octmap_path
  "moveit_visual_tools"
  "moveit_ros_planning_interface"
  "rclcpp"
  "trajectory_msgs"
  "common_services_package"
  "cv_bridge"
  "geometry_msgs"
  "pcl_conversions"
)

install(TARGETS ur5_moveit_test
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS gripper_close_moveit_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gripper_open_moveit_test
  DESTINATION lib/${PROJECT_NAME})
 
install(TARGETS ur5_moveit_test_launch
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ur5_moveit_test_object_detection
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ur5_read_positions
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ur5_pickup_sensor
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ur5_drop_sensor
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ur5_probe_sensor
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ur5_probe_sensor_service
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ur5_move_to_home
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ur5_move_to_camera
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ur5_image_octmap_path
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
