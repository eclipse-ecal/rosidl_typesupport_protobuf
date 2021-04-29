@{
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
}@
// generated from rosidl_typesupport_protobuf_cpp/resource/idl__rosidl_typesupport_protobuf_cpp.hpp.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__rosidl_typesupport_protobuf_cpp.hpp files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@
#pragma once

@{

include_directives = set()

#######################################################################
# Handle message
#######################################################################
from rosidl_parser.definition import Message
for message in content.get_elements_of_type(Message):
    TEMPLATE(
        'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, 
        interface_path=interface_path, 
        message=message,
        include_directives=include_directives)

#######################################################################
# Handle service
#######################################################################
from rosidl_parser.definition import Service
for service in content.get_elements_of_type(Service):
    TEMPLATE(
        'srv__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, 
        interface_path=interface_path, 
        service=service,
        include_directives=include_directives)

#######################################################################
# Handle action
#######################################################################
from rosidl_parser.definition import Action
for action in content.get_elements_of_type(Action):
    TEMPLATE(
        'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, interface_path=interface_path, message=action.goal,
        include_directives=include_directives)
    TEMPLATE(
        'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, interface_path=interface_path, message=action.result,
        include_directives=include_directives)
    TEMPLATE(
        'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, interface_path=interface_path, message=action.feedback,
        include_directives=include_directives)
    TEMPLATE(
        'srv__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, interface_path=interface_path, service=action.send_goal_service,
        include_directives=include_directives)
    TEMPLATE(
        'srv__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, interface_path=interface_path, service=action.get_result_service,
        include_directives=include_directives)
    TEMPLATE(
        'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
        package_name=package_name, interface_path=interface_path, message=action.feedback_message,
        include_directives=include_directives)
}@