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
@{
from rosidl_parser.definition import *
from rosidl_typesupport_protobuf import PROTO_PACKAGE_POSTFIX, ros_type_from_namespaced_type, ros_type_from_namespaced_type_c, protobuf_type_from_namespaced_type, protobuf_type_from_namespaced_type_c

def isNamespacedType(type_):
    from rosidl_parser.definition import NamespacedType
    return isinstance(type_, NamespacedType)

def isNamespacedArrayType(type_):
    from rosidl_parser.definition import AbstractNestedType
    from rosidl_parser.definition import NamespacedType
    return isinstance(type_, AbstractNestedType) and isinstance(type_.value_type, NamespacedType)

def isTypeAlreadyDeclared(type_, fw_declared_types):
    from rosidl_typesupport_protobuf import ros_type_from_namespaced_type
    return ros_type_from_namespaced_type(type_, '__') in fw_declared_types

def registerDeclaredType(type_, fw_declared_types):
    from rosidl_typesupport_protobuf import ros_type_from_namespaced_type
    fw_declared_types.add(ros_type_from_namespaced_type(type_, '__'))

types_to_declare = list()
for member in message.structure.members:
    if isNamespacedType(member.type) and not isTypeAlreadyDeclared(member.type, forward_declared_types):
        types_to_declare.append(member.type)
        registerDeclaredType(member.type, forward_declared_types)
    elif isNamespacedArrayType(member.type) and not isTypeAlreadyDeclared(member.type.value_type, forward_declared_types):
        types_to_declare.append(member.type.value_type)
        registerDeclaredType(member.type.value_type, forward_declared_types)
}@
@[if len(types_to_declare) > 0]@
// forward declaration of message dependencies and their conversion functions
@[end if]@
@[for type_ in types_to_declare]@
@{
ros_type = ros_type_from_namespaced_type_c(type_)
proto_type = protobuf_type_from_namespaced_type_c(type_)
}@

@[  for ns in type_.namespaces]@
namespace @(ns)
{
@[  end for]@
namespace typesupport_protobuf_c
{

bool convert_to_proto(const @(ros_type)&, @(proto_type)&);
bool convert_to_ros(const @(proto_type)&, @(ros_type)&);

}  // namespace typesupport_protobuf_cpp
@[  for ns in reversed(type_.namespaces)]@
}  // namespace @(ns)
@[  end for]@

@[end for]@
