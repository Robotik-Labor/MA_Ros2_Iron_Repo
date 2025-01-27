cmake_minimum_required(VERSION 3.8)
project(object_creation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(object_creation_node src/object_creation.cpp)
add_executable(object_test src/test.cpp)
add_executable(object_test2 src/test2.cpp)
add_executable(object_test3 src/test3.cpp)
add_executable(object_test4 src/Object_placement.cpp)
add_executable(object_test5 src/Object_placement2.cpp)
add_executable(object_test6 src/Object_placement3.cpp)
add_executable(Transfrom_test src/Transform_Test.cpp)
install(TARGETS
  
  object_test
  object_test2
  object_test3
  object_test4
  object_test5
  object_test6
  Transfrom_test
  DESTINATION lib/${PROJECT_NAME}
)

#ament_target_dependencies(object_creation_node
#  rclcpp
#  std_msgs
#  geometry_msgs
#  moveit_ros_planning_interface
#)

ament_target_dependencies(object_test
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
)

ament_target_dependencies(object_test2
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
)

ament_target_dependencies(object_test3
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
)

ament_target_dependencies(object_test4
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
)

ament_target_dependencies(object_test5
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
)

ament_target_dependencies(object_test6
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
)

ament_target_dependencies(Transfrom_test
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
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
