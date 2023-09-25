# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_orbslam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED orbslam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(orbslam_FOUND FALSE)
  elseif(NOT orbslam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(orbslam_FOUND FALSE)
  endif()
  return()
endif()
set(_orbslam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT orbslam_FIND_QUIETLY)
  message(STATUS "Found orbslam: 0.0.0 (${orbslam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'orbslam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${orbslam_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(orbslam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${orbslam_DIR}/${_extra}")
endforeach()
