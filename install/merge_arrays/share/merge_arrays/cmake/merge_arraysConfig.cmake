# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_merge_arrays_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED merge_arrays_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(merge_arrays_FOUND FALSE)
  elseif(NOT merge_arrays_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(merge_arrays_FOUND FALSE)
  endif()
  return()
endif()
set(_merge_arrays_CONFIG_INCLUDED TRUE)

# output package information
if(NOT merge_arrays_FIND_QUIETLY)
  message(STATUS "Found merge_arrays: 0.0.0 (${merge_arrays_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'merge_arrays' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(merge_arrays_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${merge_arrays_DIR}/${_extra}")
endforeach()
