cmake_minimum_required(VERSION 3.5)

project(proto_test)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND MSVC)
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

option(SKIP_SINGLE_RMW_TESTS
  "Skip tests involving only a single RMW implementation" OFF)
option(SKIP_MULTI_RMW_TESTS
  "Skip tests involving only multiple RMW implementations" OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(osrf_testing_tools_cpp REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(rcl REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(test_msgs REQUIRED)


  include_directories(include)
  
  ament_index_get_resource(interface_files "rosidl_interfaces" "test_msgs")
  string(REPLACE "\n" ";" interface_files "${interface_files}")


  set(message_files "")
  set(service_files "")
  set(action_files "")
  foreach(interface_file ${interface_files})
    get_filename_component(interface_ns "${interface_file}" DIRECTORY)
    get_filename_component(interface_ns "${interface_ns}" NAME)
    string_ends_with("${interface_file}" ".msg" is_message)
    if(is_message AND interface_ns STREQUAL "msg")
      list(APPEND message_files "${interface_file}")
      continue()
    endif()
    string_ends_with("${interface_file}" ".srv" is_service)
    if(is_service AND interface_ns STREQUAL "srv")
      list(APPEND service_files "${interface_file}")
      continue()
    endif()
    string_ends_with("${interface_file}" ".idl" is_action)
    if(is_action AND interface_ns STREQUAL "action")
      list(APPEND action_files "${interface_file}")
      continue()
    endif()
  endforeach()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # # Provides PYTHON_EXECUTABLE_DEBUG
  # find_package(python_cmake_module REQUIRED)
  # find_package(PythonExtra REQUIRED)

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations2)
  foreach(rmw_implementation ${rmw_implementations2})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  function(custom_test target with_message_argument)
    if(with_message_argument)
      # adding test for each message type
      foreach(message_file ${message_files})
        get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
        ament_add_test(
          "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
          COMMAND "$<TARGET_FILE:${target}>" "${TEST_MESSAGE_TYPE}"
          TIMEOUT 15
          GENERATE_RESULT_FOR_RETURN_CODE_ZERO
          APPEND_LIBRARY_DIRS "${append_library_dirs}")
        set_tests_properties(
          "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
          PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}>"
        )
      endforeach()
    else()
      ament_add_test(
        "${target}${target_suffix}"
        COMMAND "$<TARGET_FILE:${target}>"
        TIMEOUT 15
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        APPEND_LIBRARY_DIRS "${append_library_dirs}")
      set_tests_properties(
        "${target}${target_suffix}"
        PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}>"
      )
    endif()
  endfunction()

  function(custom_executable  target)
    add_executable(${target} ${ARGN})
    ament_target_dependencies(${target}
      "rclcpp"
      "rclcpp_action"
      "test_msgs"
    )
  endfunction()
 
  add_library(subscribe_types STATIC
   "test/subscribe_array_types.cpp"
   "test/subscribe_basic_types.cpp"
   "test/subscribe_string_types.cpp")
  ament_target_dependencies(subscribe_types
   "rclcpp"
   "test_msgs")

  # publisher combined with a subscriber
  custom_executable(test_proto_typeadapt_cpp
    "test/test_proto_typeadapt.cpp")
  target_link_libraries(test_proto_typeadapt_cpp subscribe_types rcpputils::rcpputils)


  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  # finding gtest once in the highest scope
  # prevents finding it repeatedly in each local scope
  ament_find_gtest()

  custom_test(test_proto_typeadapt_cpp TRUE)

endif()  # BUILD_TESTING



ament_auto_package()