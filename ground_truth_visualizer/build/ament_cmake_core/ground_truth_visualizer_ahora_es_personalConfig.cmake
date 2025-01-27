# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ground_truth_visualizer_ahora_es_personal_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ground_truth_visualizer_ahora_es_personal_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ground_truth_visualizer_ahora_es_personal_FOUND FALSE)
  elseif(NOT ground_truth_visualizer_ahora_es_personal_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ground_truth_visualizer_ahora_es_personal_FOUND FALSE)
  endif()
  return()
endif()
set(_ground_truth_visualizer_ahora_es_personal_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ground_truth_visualizer_ahora_es_personal_FIND_QUIETLY)
  message(STATUS "Found ground_truth_visualizer_ahora_es_personal: 0.0.0 (${ground_truth_visualizer_ahora_es_personal_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ground_truth_visualizer_ahora_es_personal' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ground_truth_visualizer_ahora_es_personal_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ground_truth_visualizer_ahora_es_personal_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ground_truth_visualizer_ahora_es_personal_DIR}/${_extra}")
endforeach()
