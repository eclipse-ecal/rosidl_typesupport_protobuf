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
import rosidl_parser.parser as rosidl
from rosidl_parser.definition import *
from rosidl_typesupport_protobuf import *

ros_type_ns = ros_type_namespace(package_name, interface_path)
ros_type_name = ros_type_name(message)
ros_type = ros_type(package_name, interface_path, message)
proto_type = protobuf_type(package_name, interface_path, message)

system_header_files = [
    'string',
    'algorithm'
]

header_files = [
    'rosidl_typesupport_cpp/message_type_support.hpp',
    'rosidl_typesupport_protobuf_cpp/identifier.hpp',
    'rosidl_typesupport_protobuf_cpp/wstring_conversion.hpp',
    'rosidl_typesupport_protobuf/message_type_support.hpp',
    'rosidl_typesupport_protobuf/message_type_support_decl.hpp',
    'rosidl_typesupport_protobuf/proto_descriptor_helper.hpp',
    visibility_control_header(package_name),
    protobuf_message_header(package_name, interface_path)
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
    'tmpl_forward_declarations.em',
    message=message,
    forward_declared_types=forward_declared_types
)
}@
@[  for ns in message.structure.namespaced_type.namespaces]@
namespace @(ns)
{
@[  end for]@
namespace typesupport_protobuf_cpp
{

ROSIDL_TYPESUPPORT_PROTOBUF_CPP_PUBLIC__@(package_name)
bool convert_to_proto(const @(ros_type) &ros_msg, @(proto_type) &pb_msg)
{
@[for member in message.structure.members]@
  // Member: @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
  *pb_msg.mutable_@(member.name)() = { ros_msg.@(member.name).begin(), ros_msg.@(member.name).end() };
@[    elif isinstance(member.type.value_type, AbstractWString)]@
  for(auto &member : ros_msg.@(member.name))
  {
    ::rosidl_typesupport_protobuf_cpp::write_to_string(member, *pb_msg.add_@(member.name)());
  }
@[    elif isinstance(member.type.value_type, NamespacedType)]@
  for(auto &member : ros_msg.@(member.name))
  {
    auto ptr{pb_msg.add_@(member.name)()};
    @("::" + "::".join(member.type.value_type.namespaces))::typesupport_protobuf_cpp::convert_to_proto(member, *ptr);
  }
@[    end if]@
@[  elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
  pb_msg.set_@(member.name)(ros_msg.@(member.name));
@[  elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractWString)]@
  ::rosidl_typesupport_protobuf_cpp::write_to_string(ros_msg.@(member.name), *pb_msg.mutable_@(member.name)());
@[  elif isinstance(member.type, NamespacedType)]@
  @("::" + "::".join(member.type.namespaces))::typesupport_protobuf_cpp::convert_to_proto(ros_msg.@(member.name), *pb_msg.mutable_@(member.name)());
@[  end if]@
@[end for]@

  return true;
}

ROSIDL_TYPESUPPORT_PROTOBUF_CPP_PUBLIC__@(package_name)
bool convert_to_ros(const @(proto_type) &pb_msg, @(ros_type) &ros_msg)
{
@[for member in message.structure.members]@
  // Member: @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type.value_type, BasicType) or isinstance(member.type.value_type, AbstractString)]@
@[      if isinstance(member.type, Array)]@
  std::copy_n(pb_msg.@(member.name)().begin(), pb_msg.@(member.name)().size(), ros_msg.@(member.name).begin());
@[      else]@
@[        if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == BOOLEAN_TYPE]@
  for(auto val : pb_msg.@(member.name)())
  {
    ros_msg.@(member.name).push_back(val);
  }
@[        else]@
  ros_msg.@(member.name) = { pb_msg.@(member.name)().begin(), pb_msg.@(member.name)().end() };
@[        end if]@
@[      end if]@
@[    elif isinstance(member.type.value_type, AbstractWString)]@
  {
@[      if not isinstance(member.type, Array)]@
    auto size{pb_msg.@(member.name)_size()};
    ros_msg.@(member.name).resize(size);

@[    end if]@
    auto& data{pb_msg.@(member.name)()};
    auto& ros_data{ros_msg.@(member.name)};
    int i{0};
    for(auto &str : data)
    {
      ::rosidl_typesupport_protobuf_cpp::write_to_u16string(str, ros_data[i]);
      i++;
    }
  }
@[    elif isinstance(member.type.value_type, NamespacedType)]@
  {
@[      if not isinstance(member.type, Array)]@
    auto size{pb_msg.@(member.name)_size()};
    ros_msg.@(member.name).resize(size);

@[    end if]@
    auto& data{pb_msg.@(member.name)()};
    auto& ros_data{ros_msg.@(member.name)};
    int i{0};
    for(auto &member : data)
    {
      @("::" + "::".join(member.type.value_type.namespaces))::typesupport_protobuf_cpp::convert_to_ros(member, ros_data[i]);
      i++;
    }
  }
@[    end if]@
@[  elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractString)]@
  ros_msg.@(member.name) = pb_msg.@(member.name)();
@[  elif isinstance(member.type, BasicType) or isinstance(member.type, AbstractWString)]@
  ::rosidl_typesupport_protobuf_cpp::write_to_u16string(pb_msg.@(member.name)(), ros_msg.@(member.name));
@[  elif isinstance(member.type, NamespacedType)]@
  @("::" + "::".join(member.type.namespaces))::typesupport_protobuf_cpp::convert_to_ros(pb_msg.@(member.name)(), ros_msg.@(member.name));
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
  "@(ros_type_ns)",
  "@(ros_type_name)",
  _@(ros_type_name)__serialize,
  _@(ros_type_name)__deserialize,
  _@(ros_type_name)__get_descriptor
};

@{
handle_name = "_" + ros_type_name + "__handle"
namespaced_handle_name = "::".join([ros_type_ns, "typesupport_protobuf_cpp", handle_name])
}@
static rosidl_message_type_support_t @(handle_name) = {
  rosidl_typesupport_protobuf_cpp::identifier,
  &@(type_support_name),
  get_message_typesupport_handle_function
};

}  // namespace typesupport_protobuf_cpp
@[  for ns in reversed(message.structure.namespaced_type.namespaces)]@
}  // namespace @(ns)
@[  end for]@

namespace rosidl_typesupport_protobuf
{

template<>
ROSIDL_TYPESUPPORT_PROTOBUF_CPP_EXPORT__@(package_name)
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_protobuf_cpp, @(ros_type_ns.replace("::", ", ")), @(ros_type_name))() {
  return &@(namespaced_handle_name);
}

#ifdef __cplusplus
}
#endif
