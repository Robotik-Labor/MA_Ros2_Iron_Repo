# CMake generated Testfile for 
# Source directory: /home/buhrmann/ws_moveit/src/moveit2/moveit_setup_assistant/moveit_setup_controllers
# Build directory: /home/buhrmann/ws_moveit/build/moveit_setup_controllers
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_controllers]=] "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/buhrmann/ws_moveit/build/moveit_setup_controllers/test_results/moveit_setup_controllers/test_controllers.gtest.xml" "--package-name" "moveit_setup_controllers" "--output-file" "/home/buhrmann/ws_moveit/build/moveit_setup_controllers/ament_cmake_gtest/test_controllers.txt" "--command" "/home/buhrmann/ws_moveit/build/moveit_setup_controllers/test_controllers" "--gtest_output=xml:/home/buhrmann/ws_moveit/build/moveit_setup_controllers/test_results/moveit_setup_controllers/test_controllers.gtest.xml")
set_tests_properties([=[test_controllers]=] PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/buhrmann/ws_moveit/build/moveit_setup_controllers/test_controllers" TIMEOUT "60" WORKING_DIRECTORY "/home/buhrmann/ws_moveit/build/moveit_setup_controllers" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/iron/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/buhrmann/ws_moveit/src/moveit2/moveit_setup_assistant/moveit_setup_controllers/CMakeLists.txt;48;ament_add_gtest;/home/buhrmann/ws_moveit/src/moveit2/moveit_setup_assistant/moveit_setup_controllers/CMakeLists.txt;0;")
subdirs("gtest")
