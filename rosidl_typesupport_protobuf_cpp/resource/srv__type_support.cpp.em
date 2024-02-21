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
@# Included from rosidl_typesupport_protobuf_cpp/resource/idl__type_support.cpp.em
@{
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_typesupport_protobuf import ros_service_name, ros_service_namespace, ros_service_type 

system_header_files = []
header_files = [
    'rosidl_typesupport_protobuf_cpp/identifier.hpp',
    'rosidl_typesupport_protobuf/service_type_support.hpp',
    'rosidl_typesupport_protobuf/service_type_support_decl.hpp'
]

service_name = ros_service_name(service)
service_namespace = ros_service_namespace(package_name, interface_path, '::')
service_type = ros_service_type(package_name, interface_path, service, '::')

}@
@{
TEMPLATE(
    'tmpl_include_directories.em',
    header_files=header_files,
    system_header_files = system_header_files,
    include_directives=include_directives
)
}@

@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, 
    interface_path=interface_path, 
    message=service.request_message,
    include_directives=include_directives,
    forward_declared_types = forward_declared_types)
}@
@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, 
    interface_path=interface_path, 
    message=service.response_message,
    include_directives=include_directives,
    forward_declared_types = forward_declared_types)
}@

@[for ns in service.namespaced_type.namespaces]@
namespace @(ns)
{
@[end for]@
namespace typesupport_protobuf_cpp
{

static rosidl_typesupport_protobuf::service_type_support_t _@(service_name)__callbacks = {
  "@(service_namespace)",
  "@(service_name)",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_protobuf_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name)_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_protobuf_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name)_Response)(),
};

static rosidl_service_type_support_t _@(service.namespaced_type.name)__handle = {
  rosidl_typesupport_protobuf_cpp::identifier,
  &_@(service_name)__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_protobuf_cpp
@[for ns in reversed(service.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[end for]@

namespace rosidl_typesupport_protobuf
{

template<>
ROSIDL_TYPESUPPORT_PROTOBUF_CPP_EXPORT__@(package_name)
const rosidl_service_type_support_t *
get_service_type_support_handle<@(service_type)>()
{
  return &@(service_namespace)::typesupport_protobuf_cpp::_@(service_name)__handle;
}

}  // namespace rosidl_typesupport_protobuf

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_protobuf_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name))() {
  return &@(service_namespace)::typesupport_protobuf_cpp::_@(service_name)__handle;
}

#ifdef __cplusplus
}
#endif
