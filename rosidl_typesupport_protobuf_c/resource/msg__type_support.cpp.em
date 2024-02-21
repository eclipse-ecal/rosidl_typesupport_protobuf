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
@# Included from rosidl_typesupport_protobuf_c/resource/idl__type_support.c.em
@{
import rosidl_parser.parser as rosidl
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import *
from rosidl_typesupport_protobuf import *

ros_type_ns = ros_type_namespace(package_name, interface_path, '__')
ros_type_name = ros_type_name(message)
ros_type = ros_type(package_name, interface_path, message, '__')
proto_type = protobuf_type(package_name, interface_path, message)

system_header_files = [
    'string',
    'algorithm'
]

header_files = [
    'rosidl_typesupport_cpp/message_type_support.hpp',
    visibility_control_header(package_name, 'rosidl_typesupport_protobuf_c'),
    'rosidl_typesupport_protobuf_c/identifier.hpp',
    'rosidl_typesupport_protobuf_c/to_ros_c_string.hpp',
    'rosidl_typesupport_protobuf_c/wstring_conversion.hpp',
    'rosidl_typesupport_protobuf/rosidl_generator_c_pkg_adapter.hpp',
    'rosidl_typesupport_protobuf/message_type_support.hpp',
    'rosidl_typesupport_protobuf/message_type_support_decl.hpp',
    'rosidl_typesupport_protobuf/proto_descriptor_helper.hpp',
    protobuf_message_header(package_name, interface_path)
]

for member in message.structure.members:
    keys = set([])
    type_ = member.type

    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, NamespacedType):
        if (
            type_.name.endswith(ACTION_GOAL_SUFFIX) or
            type_.name.endswith(ACTION_RESULT_SUFFIX) or
            type_.name.endswith(ACTION_FEEDBACK_SUFFIX)
        ):
            typename = type_.name.rsplit('_', 1)[0]
        else:
            typename = type_.name

        keys.add(ros_message_functions_header_c_from_namespace(type_.namespaces, typename))

    for key in keys:
      header_files.append(key)

    header_files.sort();

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
    'tmpl_forward_declarations.em',
    message=message,
    forward_declared_types=forward_declared_types
)
}@
@[  for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{
@[  end for]@
namespace typesupport_protobuf_c
{

ROSIDL_TYPESUPPORT_PROTOBUF_C_PUBLIC__@(package_name)
bool convert_to_proto(const @(ros_type) &ros_msg, @(proto_type) &pb_msg)
{
@[for member in message.structure.members]@
  // Member: @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type, Array)]@
@[      if isinstance(member.type.value_type, BasicType)]@
  *pb_msg.mutable_@(member.name)() = { std::begin(ros_msg.@(member.name)), std::end(ros_msg.@(member.name)) };
@[      elif isinstance(member.type.value_type, AbstractString)]@
  for(auto &data : ros_msg.@(member.name))
  {
    auto &str{*pb_msg.add_@(member.name)()};
    str = data.data;
  }
@[      elif isinstance(member.type.value_type, AbstractWString)]@
  for(auto &data : ros_msg.@(member.name))
  {
    auto &str{*pb_msg.add_@(member.name)()};
    ::rosidl_typesupport_protobuf_c::write_to_string(data, str);
  }
@[      elif isinstance(member.type.value_type, NamespacedType)]@
  for(auto &data : ros_msg.@(member.name))
  {
    auto ptr{pb_msg.add_@(member.name)()};
    @("::" + "::".join(member.type.value_type.namespaces))::typesupport_protobuf_c::convert_to_proto(data, *ptr);
  }
@[      end if]@
@[    elif isinstance(member.type, AbstractSequence)]@
  {
    auto size{ros_msg.@(member.name).size};
    auto data{ros_msg.@(member.name).data};
    auto arr_ptr{pb_msg.mutable_@(member.name)()};
@[      if isinstance(member.type.value_type, BasicType)]@
@{is_string = member.type.value_type.typename in [*CHARACTER_TYPES, OCTET_TYPE, 'uint8', 'int8']}@
@[        if is_string]@
    arr_ptr->insert(arr_ptr->end(), data, data + size);
@[        else]
    arr_ptr->Reserve(size);
    for(size_t i{0}; i < size; i++)
    {
      arr_ptr->AddAlreadyReserved(data[i]);
    }
@[        end if]@
@[      elif isinstance(member.type.value_type, AbstractString)]@
    arr_ptr->Reserve(size);
    for(size_t i{0}; i < size; i++)
    {
      pb_msg.add_@(member.name)(std::string{data[i].data, data[i].size});
    }
@[      elif isinstance(member.type.value_type, AbstractWString)]@
    arr_ptr->Reserve(size);
    for(size_t i{0}; i < size; i++)
    {
      ::rosidl_typesupport_protobuf_c::write_to_string(data[i], *arr_ptr->Add());
    }
@[      elif isinstance(member.type.value_type, NamespacedType)]@
    arr_ptr->Reserve(size);
    for(size_t i{0}; i < size; i++)
    {
      auto obj{arr_ptr->Add()};
      @("::" + "::".join(member.type.value_type.namespaces))::typesupport_protobuf_c::convert_to_proto(data[i], *obj);
    }
@[      end if]@
  }
@[    end if]@
@[  elif isinstance(member.type, BasicType)]@
  pb_msg.set_@(member.name)(ros_msg.@(member.name));
@[  elif isinstance(member.type, AbstractString)]@
  pb_msg.set_@(member.name)(ros_msg.@(member.name).data);
@[  elif isinstance(member.type, AbstractWString)]@
  ::rosidl_typesupport_protobuf_c::write_to_string(ros_msg.@(member.name), *pb_msg.mutable_@(member.name)());
@[  elif isinstance(member.type, NamespacedType)]@
  @("::" + "::".join(member.type.namespaces))::typesupport_protobuf_c::convert_to_proto(ros_msg.@(member.name), *pb_msg.mutable_@(member.name)());
@[  end if]@
@[end for]@

  return true;
}

ROSIDL_TYPESUPPORT_PROTOBUF_C_PUBLIC__@(package_name)
bool convert_to_ros(const @(proto_type) &pb_msg, @(ros_type) &ros_msg)
{
@[for member in message.structure.members]@
  // Member: @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type, Array)]@
  {
    auto &pb_arr{pb_msg.@(member.name)()};

@[      if isinstance(member.type.value_type, BasicType)]@
    std::copy(pb_arr.begin(), pb_arr.end(), ros_msg.@(member.name));
@[      elif isinstance(member.type.value_type, AbstractString)]@
    for(size_t i{0}; i < sizeof(ros_msg.@(member.name))/sizeof(ros_msg.@(member.name)[0]); i++)
    {
      ::typesupport_protobuf_c::to_ros_c_string(pb_arr.Get(i), ros_msg.@(member.name)[i]);
    }
@[      elif isinstance(member.type.value_type, AbstractWString)]@
    for(size_t i{0}; i < sizeof(ros_msg.@(member.name))/sizeof(ros_msg.@(member.name)[0]); i++)
    {
      ::rosidl_typesupport_protobuf_c::write_to_u16string(pb_arr.Get(i), ros_msg.@(member.name)[i]);
    }
@[      elif isinstance(member.type.value_type, NamespacedType)]@
    for(size_t i{0}; i < sizeof(ros_msg.@(member.name))/sizeof(ros_msg.@(member.name)[0]); i++)
    {
      @("::" + "::".join(member.type.value_type.namespaces))::typesupport_protobuf_c::convert_to_ros(pb_arr.Get(i), ros_msg.@(member.name)[i]);
    }
@[      end if]@
  }
@[    elif isinstance(member.type, AbstractSequence)]@
@{
value_type_ = member.type.value_type
if isinstance(value_type_, AbstractString):
    array_init = 'rosidl_runtime_c__String__Sequence__init'
    array_fini = 'rosidl_runtime_c__String__Sequence__fini'
elif isinstance(value_type_, AbstractWString):
    array_init = 'rosidl_runtime_c__U16String__Sequence__init'
    array_fini = 'rosidl_runtime_c__U16String__Sequence__fini'
elif isinstance(value_type_, BasicType):
    type_ = value_type_.typename
    type_ = type_.replace(' ', '_')
    array_init = 'rosidl_runtime_c__{type_}__Sequence__init'.format(**locals())
    array_fini = 'rosidl_runtime_c__{type_}__Sequence__fini'.format(**locals())
else:
    type_ = value_type_
    array_init = '__'.join(type_.namespaced_name()) + '__Sequence__init'
    array_fini = '__'.join(type_.namespaced_name()) + '__Sequence__fini'
}@
  {
    auto &pb_arr{pb_msg.@(member.name)()};
    auto size{pb_arr.size()};

    @(array_init)(&ros_msg.@(member.name), size);
    auto ros_arr{ros_msg.@(member.name).data};

@[      if isinstance(member.type.value_type, BasicType)]@
    std::copy(pb_arr.begin(), pb_arr.end(), ros_arr);
@[      elif isinstance(member.type.value_type, AbstractString)]@
    for(int i{0}; i < size; i++)
    {
      ::typesupport_protobuf_c::to_ros_c_string(pb_arr.Get(i), ros_arr[i]);
    }
@[      elif isinstance(member.type.value_type, AbstractWString)]@
    for(int i{0}; i < size; i++)
    {
      ::rosidl_typesupport_protobuf_c::write_to_u16string(pb_arr.Get(i), ros_arr[i]);
    }
@[      elif isinstance(member.type.value_type, NamespacedType)]@
    for(int i{0}; i < size; i++)
    {
      @("::" + "::".join(member.type.value_type.namespaces))::typesupport_protobuf_c::convert_to_ros(pb_arr.Get(i), ros_arr[i]);
    }
@[      end if]@
  }
@[    end if]@
@[  elif isinstance(member.type, BasicType)]@
  ros_msg.@(member.name) = pb_msg.@(member.name)();
@[  elif isinstance(member.type, AbstractString)]@
  ::typesupport_protobuf_c::to_ros_c_string(pb_msg.@(member.name)(), ros_msg.@(member.name));
@[  elif isinstance(member.type, AbstractWString)]@
  ::rosidl_typesupport_protobuf_c::write_to_u16string(pb_msg.@(member.name)(), ros_msg.@(member.name));
@[  elif isinstance(member.type, NamespacedType)]@
  @("::" + "::".join(member.type.namespaces))::typesupport_protobuf_c::convert_to_ros(pb_msg.@(member.name)(), ros_msg.@(member.name));
@[  end if]@
@[end for]@

  return true;
}

static bool _@(ros_type_name)__serialize(const void *untyped_ros_msg, std::string &serialized_msg)
{
  @(proto_type) pb_msg{};
  auto ros_msg{static_cast<const @(ros_type) *>(untyped_ros_msg)};

  convert_to_proto(*ros_msg, pb_msg);
  return pb_msg.SerializeToString(&serialized_msg);
}

static bool _@(ros_type_name)__deserialize(void *untyped_ros_msg, const void *serialized_msg, size_t size)
{
  @(proto_type) pb_msg{};
  auto ros_msg{static_cast<@(ros_type) *>(untyped_ros_msg)};

  pb_msg.ParseFromArray(serialized_msg, size);
  return convert_to_ros(pb_msg, *ros_msg);
}

static std::string _@(ros_type_name)__get_descriptor()
{
  return "";
  return GetProtoMessageDescription<@(proto_type)>();
}

@{type_support_name = "_" + ros_type_name + "__type_support"}@
static rosidl_typesupport_protobuf::message_type_support_t @(type_support_name) = {
  "@(ros_type_ns.replace("__", "::"))",
  "@(ros_type_name)",
  _@(ros_type_name)__serialize,
  _@(ros_type_name)__deserialize,
  _@(ros_type_name)__get_descriptor
};

@{
handle_name = "_" + ros_type_name + "__handle"
namespaced_handle_name = "::".join([ros_type_ns.replace("__", "::"), "typesupport_protobuf_c", handle_name])
}@
static rosidl_message_type_support_t @(handle_name) = {
  rosidl_typesupport_protobuf_c::identifier,
  &@(type_support_name),
  get_message_typesupport_handle_function
};

}  // namespace typesupport_protobuf_c
@[  for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[  end for]@

namespace rosidl_typesupport_protobuf
{

template<>
ROSIDL_TYPESUPPORT_PROTOBUF_C_EXPORT__@(package_name)
const rosidl_message_type_support_t *get_message_type_support_handle<@(ros_type)>()
{
  return &@(namespaced_handle_name);
}

}  // namespace rosidl_typesupport_protobuf

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_protobuf_c, @(ros_type_ns.replace("__", ", ")), @(ros_type_name))() {
  return &@(namespaced_handle_name);
}

#ifdef __cplusplus
}
#endif
