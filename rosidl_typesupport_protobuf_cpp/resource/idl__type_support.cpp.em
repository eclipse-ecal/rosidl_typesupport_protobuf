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
// generated from rosidl_typesupport_protobuf_cpp/resource/idl__type_support.cpp.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__type_support.cpp files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@
@{
from rosidl_typesupport_protobuf import typesupport_header, ros_message_header
from rosidl_cmake import convert_camel_case_to_lower_case_underscore

include_directives = set()
forward_declared_types = set()

system_header_files = []
header_files = [
  typesupport_header(package_name, interface_path, 'rosidl_typesupport_protobuf_cpp'), 
  ros_message_header(package_name, interface_path)
]
}@

@{
TEMPLATE(
    'tmpl_include_directories.em',
    header_files=header_files,
    system_header_files=system_header_files,
    include_directives=include_directives
)
}@

@{
#######################################################################
# Handle message
#######################################################################
from rosidl_parser.definition import Message
for message in content.get_elements_of_type(Message):
    TEMPLATE(
        'msg__type_support.cpp.em',
        package_name=package_name, 
        interface_path=interface_path, 
        message=message,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)

#######################################################################
# Handle service
#######################################################################
from rosidl_parser.definition import Service
for service in content.get_elements_of_type(Service):
    TEMPLATE(
        'srv__type_support.cpp.em',
        package_name=package_name, 
        interface_path=interface_path, 
        service=service,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)

#######################################################################
# Handle action
#######################################################################
from rosidl_parser.definition import Action
for action in content.get_elements_of_type(Action):
    TEMPLATE(
        'msg__type_support.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.goal,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)
    TEMPLATE(
        'msg__type_support.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.result,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)
    TEMPLATE(
        'msg__type_support.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.feedback,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)
    TEMPLATE(
        'srv__type_support.cpp.em',
        package_name=package_name, interface_path=interface_path, service=action.send_goal_service,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)
    TEMPLATE(
        'srv__type_support.cpp.em',
        package_name=package_name, interface_path=interface_path, service=action.get_result_service,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)
    TEMPLATE(
        'msg__type_support.cpp.em',
        package_name=package_name, interface_path=interface_path, message=action.feedback_message,
        include_directives=include_directives,
        forward_declared_types = forward_declared_types)
}@