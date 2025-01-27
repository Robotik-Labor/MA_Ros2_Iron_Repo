# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_picknik_accessories_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED picknik_accessories_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(picknik_accessories_FOUND FALSE)
  elseif(NOT picknik_accessories_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(picknik_accessories_FOUND FALSE)
  endif()
  return()
endif()
set(_picknik_accessories_CONFIG_INCLUDED TRUE)

# output package information
if(NOT picknik_accessories_FIND_QUIETLY)
  message(STATUS "Found picknik_accessories: 6.0.0 (${picknik_accessories_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'picknik_accessories' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT picknik_accessories_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(picknik_accessories_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${picknik_accessories_DIR}/${_extra}")
endforeach()
