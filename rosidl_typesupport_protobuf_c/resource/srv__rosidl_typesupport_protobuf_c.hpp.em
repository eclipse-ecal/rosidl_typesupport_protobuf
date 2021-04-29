@# Included from rosidl_typesupport_protobuf_c/resource/idl__rosidl_typesupport_protobuf_c.hpp.em
@{
system_header_files = []
header_files = [
    'rmw/types.h',
    'rosidl_typesupport_cpp/service_type_support.hpp',
    'rosidl_typesupport_interface/macros.h',
    'rosidl_typesupport_protobuf/visibility_control.h',
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
    'msg__rosidl_typesupport_protobuf_c.hpp.em',
    package_name=package_name, 
    interface_path=interface_path,
    message=service.request_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__rosidl_typesupport_protobuf_c.hpp.em',
    package_name=package_name, 
    interface_path=interface_path, 
    message=service.response_message,
    include_directives=include_directives)
}@

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_PROTOBUF_PUBLIC
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_protobuf_c, @(', '.join([package_name] + list(interface_path.parents[0].parts) + [service.namespaced_type.name])))();

#ifdef __cplusplus
}
#endif
