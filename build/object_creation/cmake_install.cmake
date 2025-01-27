# Install script for directory: /home/buhrmann/ws_moveit/src/object_creation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/buhrmann/ws_moveit/install/object_creation")
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/object_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/object_test2")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test2")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/object_test3")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test3")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/object_test4")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test4")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/object_test5")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test5")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/object_test6")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/object_test6")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/object_creation" TYPE EXECUTABLE FILES "/home/buhrmann/ws_moveit/build/object_creation/Transfrom_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test"
         OLD_RPATH "/home/buhrmann/ws_moveit/install/moveit_ros_planning_interface/lib:/home/buhrmann/ws_moveit/install/moveit_visual_tools/lib:/home/buhrmann/ws_moveit/install/moveit_ros_move_group/lib:/opt/ros/iron/lib/x86_64-linux-gnu:/opt/ros/iron/lib:/home/buhrmann/ws_moveit/install/moveit_ros_warehouse/lib:/home/buhrmann/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/iron/opt/rviz_ogre_vendor/lib:/home/buhrmann/ws_moveit/install/moveit_ros_planning/lib:/home/buhrmann/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/buhrmann/ws_moveit/install/moveit_core/lib:/home/buhrmann/ws_moveit/install/moveit_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/object_creation/Transfrom_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/object_creation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/object_creation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation/environment" TYPE FILE FILES "/opt/ros/iron/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation/environment" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation/environment" TYPE FILE FILES "/opt/ros/iron/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation/environment" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_index/share/ament_index/resource_index/packages/object_creation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation/cmake" TYPE FILE FILES
    "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_core/object_creationConfig.cmake"
    "/home/buhrmann/ws_moveit/build/object_creation/ament_cmake_core/object_creationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_creation" TYPE FILE FILES "/home/buhrmann/ws_moveit/src/object_creation/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/buhrmann/ws_moveit/build/object_creation/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
