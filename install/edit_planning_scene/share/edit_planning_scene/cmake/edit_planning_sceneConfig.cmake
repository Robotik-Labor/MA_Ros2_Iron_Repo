# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_edit_planning_scene_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED edit_planning_scene_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(edit_planning_scene_FOUND FALSE)
  elseif(NOT edit_planning_scene_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(edit_planning_scene_FOUND FALSE)
  endif()
  return()
endif()
set(_edit_planning_scene_CONFIG_INCLUDED TRUE)

# output package information
if(NOT edit_planning_scene_FIND_QUIETLY)
  message(STATUS "Found edit_planning_scene: 0.0.0 (${edit_planning_scene_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'edit_planning_scene' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT edit_planning_scene_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(edit_planning_scene_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${edit_planning_scene_DIR}/${_extra}")
endforeach()
