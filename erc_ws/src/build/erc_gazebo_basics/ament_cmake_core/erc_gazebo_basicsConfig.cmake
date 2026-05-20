# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_erc_gazebo_basics_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED erc_gazebo_basics_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(erc_gazebo_basics_FOUND FALSE)
  elseif(NOT erc_gazebo_basics_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(erc_gazebo_basics_FOUND FALSE)
  endif()
  return()
endif()
set(_erc_gazebo_basics_CONFIG_INCLUDED TRUE)

# output package information
if(NOT erc_gazebo_basics_FIND_QUIETLY)
  message(STATUS "Found erc_gazebo_basics: 1.0.0 (${erc_gazebo_basics_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'erc_gazebo_basics' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT erc_gazebo_basics_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(erc_gazebo_basics_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${erc_gazebo_basics_DIR}/${_extra}")
endforeach()
