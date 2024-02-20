# ================================= Apache 2.0 =================================
#
# Copyright (C) 2021 Continental
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ================================= Apache 2.0 =================================

find_package(Protobuf REQUIRED)

find_package(rosidl_adapter_proto REQUIRED)
find_package(rosidl_typesupport_protobuf REQUIRED)

set(_output_path "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_protobuf_cpp/${PROJECT_NAME}")

# Create a list of files that will be generated from each IDL file
set(_generated_files "")
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  # Turn idl name into file names
  string_camel_case_to_lower_case_underscore("${_idl_name}" _header_name)
  list(APPEND _generated_files
    "${_output_path}/${_parent_folder}/detail/${_header_name}__type_support.cpp"
    "${_output_path}/${_parent_folder}/${_header_name}__rosidl_typesupport_protobuf_cpp.hpp"
  )
endforeach()

# generate header to switch between export and import for a specific package
set(_visibility_control_file
"${_output_path}/rosidl_typesupport_protobuf_cpp__visibility_control.h")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
configure_file(
  "${rosidl_typesupport_protobuf_cpp_TEMPLATE_DIR}/rosidl_typesupport_protobuf_cpp__visibility_control.h.in"
  "${_visibility_control_file}"
  @ONLY
)

set(_target_suffix "__rosidl_typesupport_protobuf_cpp")

add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} SHARED
  ${_generated_files}
  ${rosidl_adapter_proto_GENERATED_CPP}
)

# Change output library name if asked to
if(rosidl_generate_interfaces_LIBRARY_NAME)
  set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PROPERTIES OUTPUT_NAME "${rosidl_generate_interfaces_LIBRARY_NAME}${_target_suffix}")
endif()

# set C++ standard
set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PROPERTIES CXX_STANDARD 14
)

# Set flag for visibility macro
if(WIN32)
  target_compile_definitions(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PRIVATE "ROSIDL_TYPESUPPORT_PROTOBUF_BUILDING_DLL" "ROSIDL_TYPESUPPORT_PROTOBUF_CPP_BUILDING_DLL__${PROJECT_NAME}")
  set(Protobuf_USE_STATIC_LIBS TRUE)
endif()

# Set compiler flags
if(NOT WIN32)
  set(_target_compile_flags "-Wall -Wextra -Wpedantic")
else()
  set(_target_compile_flags "/W4")
endif()
string(REPLACE ";" " " _target_compile_flags "${_target_compile_flags}")
set_target_properties(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  PROPERTIES COMPILE_FLAGS "${_target_compile_flags}"
)

# Include headers from other generators
  target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter_proto>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_protobuf_cpp>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )


ament_target_dependencies(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  rmw
  rosidl_typesupport_protobuf
  rosidl_typesupport_protobuf_cpp
  rosidl_typesupport_interface
)

# Depend on dependencies
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  ament_target_dependencies(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${_pkg_name})
  target_link_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${${_pkg_name}_LIBRARIES${_target_suffix}})
endforeach()

target_link_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix} ${Protobuf_LIBRARY})

# Make top level generation target depend on this library
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)

# Make this library depend on target created by rosidl_generator_cpp
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_cpp
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(
    DIRECTORY "${_output_path}/"
    DESTINATION "include/${PROJECT_NAME}/${PROJECT_NAME}"
    PATTERN "*.cpp" EXCLUDE
  )

  if(NOT _generated_files STREQUAL "")
    ament_export_include_directories("include/${PROJECT_NAME}")
  endif()

  # Export old-style CMake variables
  ament_export_include_directories("include/${PROJECT_NAME}")
  rosidl_export_typesupport_libraries(${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix})

  # Export modern CMake targets
  ament_export_targets(export_${rosidl_generate_interfaces_TARGET}${_target_suffix})
  rosidl_export_typesupport_targets(${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix})

  install(
    TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    EXPORT export_${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  rosidl_export_typesupport_libraries(${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix})

  ament_export_libraries(${rosidl_generate_interfaces_TARGET}${_target_suffix})
endif()