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
@# Included from rosidl_typesupport_protobuf_c/resource/idl__rosidl_typesupport_protobuf_c.hpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_typesupport_protobuf import *

system_header_files = [
    "string"
]

header_files = [ 
    "rosidl_typesupport_cpp/message_type_support.hpp",
    ros_message_header_c(package_name, interface_path),
    ros_message_header(package_name, interface_path),
    visibility_control_header(package_name, 'rosidl_typesupport_protobuf_c'),
    "rosidl_typesupport_interface/macros.h",
    protobuf_message_header(package_name, interface_path)
]

ros_type_ns = ros_type_namespace(package_name, interface_path, '__')
ros_type_name = ros_type_name(message)
ros_type = ros_type(package_name, interface_path, message, '__')
proto_type = protobuf_type(package_name, interface_path, message)

}@

@{
TEMPLATE(
    'tmpl_include_directories.em',
    header_files=header_files,
    system_header_files=system_header_files,
    include_directives=include_directives
)
}@

@[for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{
@[end for]@
namespace typesupport_protobuf_c
{

bool
ROSIDL_TYPESUPPORT_PROTOBUF_C_PUBLIC__@(package_name)
convert_to_proto(const @(ros_type) &ros_msg, @(proto_type) &pb_msg);

bool
ROSIDL_TYPESUPPORT_PROTOBUF_C_PUBLIC__@(package_name)
convert_to_ros(const @(proto_type) &pb_msg, @(ros_type) &ros_msg);

}  // namespace typesupport_protobuf_c
@[  for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[  end for]@

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_PROTOBUF_C_PUBLIC__@(package_name)
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_protobuf_c, @(ros_type_ns.replace("__", ", ")), @(ros_type_name))();

#ifdef __cplusplus
}
#endif
