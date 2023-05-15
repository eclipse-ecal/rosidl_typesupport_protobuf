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
import rosidl_parser.definition as rosidl

from rosidl_adapter_proto import PROTO_PACKAGE_POSTFIX
from rosidl_adapter_proto import MSG_TYPE_TO_PROTO
from rosidl_adapter_proto import compute_proto_field_number

import sys
import re

}@
@#
@# ================ Message definition ================
@#
@{
comment = ""
for annotation in message.structure.annotations:
    if (annotation.name == "verbatim") \
            and ("language" in annotation.value) \
            and (annotation.value["language"] == "comment") \
            and ("text" in annotation.value):
        comment = "//" + re.sub("\n", "\n// ", annotation.value["text"])
        break
}@
@[if comment != ""]@
@(comment)
@[end if]@
message @(message.structure.namespaced_type.name)
{

@# Message fields
@{
used_field_numbers = set()
proto_member_list  = []

str_max_len_type         = 0
str_max_len_name         = 0
str_max_len_field_number = 0

for member in message.structure.members:

    idl_type = member.type

    is_repeated  = False
    is_byte_type = False
    is_char_type = False

    # Check if datatype is repeated
    if isinstance(idl_type, rosidl.AbstractNestedType):
        is_repeated = True
        idl_type = idl_type.value_type

    # Check for actual datatype
    if isinstance(idl_type, rosidl.BasicType):
        proto_type = MSG_TYPE_TO_PROTO[idl_type.typename]
        if idl_type.typename in [rosidl.OCTET_TYPE, 'uint8', 'int8']:
            is_byte_type = True
        elif idl_type.typename in rosidl.CHARACTER_TYPES:
            is_char_type = True
    elif isinstance(idl_type, rosidl.AbstractString):
        proto_type = "string"
    elif isinstance(idl_type, rosidl.AbstractWString):
        proto_type = "bytes"
    elif isinstance(idl_type, rosidl.NamespacedType):
        proto_type = ".".join(idl_type.namespaces + [PROTO_PACKAGE_POSTFIX] + [idl_type.name])
    elif isinstance(idl_type, rosidl.NamedType):
        proto_type = idl_type.name

    # Check if we can improve the repeated field with a scalar type
    if is_repeated:
        if is_byte_type:
            proto_type = "bytes"
        elif is_char_type:
            proto_type = "string"
        else:
            proto_type = "repeated " + proto_type

    # Compute a field number of the actual variable name. This will almost always be unique.
    member_name_for_field_number = member.name
    additional_counter = 0

    field_number = compute_proto_field_number(member_name_for_field_number)

    while field_number in used_field_numbers:
        sys.stderr.write("WARNING: Field " + member.name + " maps to a protobuf field number that is already in use. This will hurt the downward compatibility of this message..\n")
        member_name_for_field_number = member.name + str(additional_counter)
        field_number = compute_proto_field_number(member_name_for_field_number)
        additional_counter += 1

    used_field_numbers.add(field_number)

    member_dict = {
                    "type":         proto_type,
                    "name":         member.name,
                    "field_number": field_number,
                }

    for annotation in member.annotations:
        if (annotation.name == "verbatim") \
                and ("language" in annotation.value) \
                and (annotation.value["language"] == "comment") \
                and ("text" in annotation.value):
            member_dict["comment"] = "  //" + re.sub("\n", "\n  //", annotation.value["text"])
        if (annotation.name == "unit") and ("value" in annotation.value):
            member_dict["unit"] = annotation.value["value"]


    proto_member_list.append(member_dict)

    str_max_len_type         = max(str_max_len_type, len(proto_type))
    str_max_len_name         = max(str_max_len_name, len(member.name))
    str_max_len_field_number = max(str_max_len_field_number, len(str(field_number)))
}@
@[for proto_member in proto_member_list]@
@[    if "comment" in proto_member]@

@(proto_member["comment"])
@[    end if]@
  @(str.ljust(proto_member["type"], str_max_len_type)) @(str.ljust(proto_member["name"], str_max_len_name)) = @(str.rjust(str(proto_member["field_number"]), str_max_len_field_number));@
@[    if "unit" in proto_member]@
 // [@(proto_member["unit"])@]@
@[    end if]
@[end for]@
}