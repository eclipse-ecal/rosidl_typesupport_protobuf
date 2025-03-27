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
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(rosidl_adapter_proto)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rosidl_cmake REQUIRED)
  ament_add_pytest_test(pytest test)
  set(generator_arguments_file "${CMAKE_CURRENT_SOURCE_DIR}/test/msg/test_rosid_adapter_proto__arguments.json")
  rosidl_write_generator_arguments(
    "${generator_arguments_file}"
    PACKAGE_NAME "${PROJECT_NAME}"
    IDL_TUPLES "${CMAKE_CURRENT_SOURCE_DIR}/test:msg/BoolTest.idl"
    OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test"
    TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resource"
    TARGET_DEPENDENCIES "${CMAKE_CURRENT_SOURCE_DIR}/test/msg/BoolTest.idl"
    ADDITIONAL_FILES "")
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package(CONFIG_EXTRAS "cmake/rosidl_adapter_proto-extras.cmake.in")

install(
  PROGRAMS bin/rosidl_adapter_proto
  DESTINATION lib/rosidl_adapter_proto
)

install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)