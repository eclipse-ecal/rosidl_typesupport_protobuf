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

find_package(PythonInterp REQUIRED)
if(NOT PYTHON_EXECUTABLE)
  message(FATAL_ERROR "Variable 'PYTHON_EXECUTABLE' must not be empty")
endif()

set(rosidl_adapter_proto_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter_proto/${PROJECT_NAME}")

# Create a list of proto directories
set(_proto_include_dirs "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  set(_proto_dir "${${_pkg_name}_DIR}/../../../share/${_pkg_name}")
  normalize_path(_proto_dir "${_proto_dir}")
  list(APPEND _proto_include_dirs "${_proto_dir}")
endforeach()

set(_target_dependencies
  "${rosidl_adapter_proto_BIN}"
  ${rosidl_adapter_proto_GENERATOR_FILES}
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_proto_include_dirs})
foreach(dep ${_target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

# Write all this to a file to work around command line length limitations on some platforms
set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter_proto__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  OUTPUT_DIR "${rosidl_adapter_proto_OUTPUT_DIR}"
  TEMPLATE_DIR "${rosidl_adapter_proto_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${_target_dependencies}
  ADDITIONAL_FILES "${_proto_include_dirs}")

execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" "${rosidl_adapter_proto_BIN}"
  --generator-arguments-file "${generator_arguments_file}"
  --protoc-path "${Protobuf_PROTOC_EXECUTABLE}"
  ERROR_VARIABLE error
  RESULT_VARIABLE result
)

if(NOT result EQUAL 0)
  message(FATAL_ERROR "Proto file generation failed, error code: ${error}")
endif()

set(rosidl_adapter_proto_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter_proto")
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  list(APPEND rosidl_adapter_proto_GENERATED_CPP "${rosidl_adapter_proto_OUTPUT_DIR}/${_parent_folder}/${_idl_name}.pb.cc")
endforeach()

# generate header to switch between export and import for a specific package
set(rosidl_adapter_proto_VISIBILITY_CONTROL_HEADER
"${rosidl_adapter_proto_OUTPUT_DIR}/rosidl_adapter_proto__visibility_control.h")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
configure_file(
  "${rosidl_adapter_proto_TEMPLATE_DIR}/rosidl_adapter_proto__visibility_control.h.in"
  "${rosidl_adapter_proto_VISIBILITY_CONTROL_HEADER}"
  @ONLY
)

install(
  DIRECTORY ${rosidl_adapter_proto_OUTPUT_DIR}
  DESTINATION "include/${PROJECT_NAME}"
  PATTERN "*.h"
)

install(
  DIRECTORY ${rosidl_adapter_proto_OUTPUT_DIR}
  DESTINATION "share/${PROJECT_NAME}"
  PATTERN "*.proto"
)