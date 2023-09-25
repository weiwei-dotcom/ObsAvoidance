# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_model3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED model3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(model3_FOUND FALSE)
  elseif(NOT model3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(model3_FOUND FALSE)
  endif()
  return()
endif()
set(_model3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT model3_FIND_QUIETLY)
  message(STATUS "Found model3: 0.0.0 (${model3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'model3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${model3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(model3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${model3_DIR}/${_extra}")
endforeach()
