# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rb_theron_description_fortress_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rb_theron_description_fortress_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rb_theron_description_fortress_FOUND FALSE)
  elseif(NOT rb_theron_description_fortress_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rb_theron_description_fortress_FOUND FALSE)
  endif()
  return()
endif()
set(_rb_theron_description_fortress_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rb_theron_description_fortress_FIND_QUIETLY)
  message(STATUS "Found rb_theron_description_fortress: 0.0.0 (${rb_theron_description_fortress_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rb_theron_description_fortress' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rb_theron_description_fortress_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rb_theron_description_fortress_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rb_theron_description_fortress_DIR}/${_extra}")
endforeach()
