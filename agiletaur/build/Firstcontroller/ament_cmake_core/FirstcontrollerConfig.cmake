# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Firstcontroller_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Firstcontroller_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Firstcontroller_FOUND FALSE)
  elseif(NOT Firstcontroller_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Firstcontroller_FOUND FALSE)
  endif()
  return()
endif()
set(_Firstcontroller_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Firstcontroller_FIND_QUIETLY)
  message(STATUS "Found Firstcontroller: 0.0.0 (${Firstcontroller_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Firstcontroller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Firstcontroller_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Firstcontroller_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Firstcontroller_DIR}/${_extra}")
endforeach()
