# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vonns2_reconstruction_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vonns2_reconstruction_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vonns2_reconstruction_FOUND FALSE)
  elseif(NOT vonns2_reconstruction_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vonns2_reconstruction_FOUND FALSE)
  endif()
  return()
endif()
set(_vonns2_reconstruction_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vonns2_reconstruction_FIND_QUIETLY)
  message(STATUS "Found vonns2_reconstruction: 0.0.0 (${vonns2_reconstruction_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vonns2_reconstruction' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vonns2_reconstruction_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vonns2_reconstruction_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vonns2_reconstruction_DIR}/${_extra}")
endforeach()
