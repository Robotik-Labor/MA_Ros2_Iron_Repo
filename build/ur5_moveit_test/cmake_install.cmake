# Install script for directory: /home/buhrmann/ws_moveit/src/ur5_moveit_test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/buhrmann/ws_moveit/install/ur5_moveit_test")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ur5_moveit_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/gripper_close_moveit_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_close_moveit_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/gripper_open_moveit_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/gripper_open_moveit_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ur5_moveit_test_launch")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_launch")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ur5_moveit_test_object_detection")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_moveit_test_object_detection")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ur5_read_positions")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_read_positions")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ur5_pickup_sensor")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_pickup_sensor")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ur5_drop_sensor")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_moveit_test/ur5_drop_sensor")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE DIRECTORY FILES "/home/buhrmann/ws_moveit/src/ur5_moveit_test/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ur5_moveit_test")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ur5_moveit_test")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test/environment" TYPE FILE FILES "/opt/ros/iron/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test/environment" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test/environment" TYPE FILE FILES "/opt/ros/iron/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test/environment" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_index/share/ament_index/resource_index/packages/ur5_moveit_test")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test/cmake" TYPE FILE FILES
    "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_core/ur5_moveit_testConfig.cmake"
    "/home/buhrmann/ws_moveit/build/ur5_moveit_test/ament_cmake_core/ur5_moveit_testConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_moveit_test" TYPE FILE FILES "/home/buhrmann/ws_moveit/src/ur5_moveit_test/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/buhrmann/ws_moveit/build/ur5_moveit_test/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
