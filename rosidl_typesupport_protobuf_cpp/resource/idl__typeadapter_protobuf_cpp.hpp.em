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
@# Included from rosidl_typesupport_protobuf_cpp/resource/idl__rosidl_typesupport_protobuf_cpp.hpp.em
@{

from rosidl_parser.definition import Message

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_typesupport_protobuf import *

system_header_files = [
    "string"
]

header_files = [
    ros_message_header(package_name, interface_path),
    visibility_control_header(package_name),
    'rosidl_typesupport_protobuf/rosidl_generator_c_pkg_adapter.hpp',
    'rosidl_typesupport_interface/macros.h',
    protobuf_message_header(package_name, interface_path)
]

include_directives = set()

}@
@{
TEMPLATE(
    'tmpl_include_directories.em',
    header_files=header_files,
    system_header_files=system_header_files,
    include_directives=include_directives
)
}@

namespace rclcpp 
{


@[for message in content.get_elements_of_type(Message)]@

@{
ros_type_ns = ros_type_namespace(package_name, interface_path)
ros_type_name = ros_type_name(message)
ros_type = ros_type(package_name, interface_path, message)
proto_type = protobuf_type(package_name, interface_path, message)
}@

  template<>
  struct TypeAdapter<@(proto_type),  @(ros_type)>
  {
    using is_specialized = std::true_type;
    using custom_type = @(proto_type);
    using ros_message_type =  @(ros_type);

    static
    void
    convert_to_ros_message(
     const custom_type & source,
     ros_message_type & destination)
    {
     @("::".join(message.structure.namespaced_type.namespaces))::typesupport_protobuf_c::convert_to_ros(source, destination);
    }

    static
    void
    convert_to_custom(
      const ros_message_type & source,
      custom_type & destination)
    {
      @("::".join(message.structure.namespaced_type.namespaces))::typesupport_protobuf_c::convert_to_proto(source, destination);
    }
  };

@[end for]@

}

