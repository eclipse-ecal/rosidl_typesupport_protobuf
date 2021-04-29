@# Included from rosidl_typesupport_protobuf_cpp/resource/idl__rosidl_typesupport_protobuf_cpp.hpp.em
@{
system_header_files = []
header_files = [
    'rosidl_typesupport_cpp/service_type_support.hpp',
    'rosidl_typesupport_interface/macros.h'
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
TEMPLATE(
    'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
    package_name=package_name, 
    interface_path=interface_path,
    message=service.request_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__rosidl_typesupport_protobuf_cpp.hpp.em',
    package_name=package_name, 
    interface_path=interface_path, 
    message=service.response_message,
    include_directives=include_directives)
}@

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_PROTOBUF_CPP_PUBLIC__@(package_name)
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_protobuf_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts) + [service.namespaced_type.name])))();

#ifdef __cplusplus
}
#endif
