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

import subprocess
import zlib

from rosidl_cmake import generate_files, convert_camel_case_to_lower_case_underscore
import rosidl_parser.definition as rosidl

# A postfix for the protobuf package name / the c++ namespace
PROTO_PACKAGE_POSTFIX = 'pb'

# The rpc-function name for service calls. As ros services can only offer a
# single function, this function gets a static name in the protobuf service
PROTO_SERVICE_CALL_NAME = 'Call'

# The rpc function name for sending an action goal
PROTO_ACTION_SEND_GOAL_CALL_NAME = 'SendGoal'

# The rpc function name for retrieving the action result
PROTO_ACTION_GET_RESULT_CALL_NAME = 'GetResult'

# A Mapping from IDL -> Protobuf type
MSG_TYPE_TO_PROTO = {
    'boolean':     'bool',
    'octet':       'uint32',
    'char':        'uint32',
    'wchar':       'uint32',
    'float':       'float',
    'double':      'double',
    'long double': 'double',
    'uint8':       'uint32',
    'int8':        'int32',
    'uint16':      'uint32',
    'int16':       'int32',
    'uint32':      'fixed32',
    'int32':       'sfixed32',
    'uint64':      'fixed64',
    'int64':       'sfixed64',
    'string':      'string',
    'wstring':     'bytes',
}

field_val = 0


def compute_proto_field_number(variable_name):
    # Field number rules (https://developers.google.com/protocol-buffers/docs/
    # proto#assigning_field_numbers)
    #
    # Smallest: 1
    # Largest:  536870911 (= 2^29 - 1)
    #
    # Reserved Range: 19000 to 19999 (=> 1000 values)

    # Create a 32 bit hash from the variable name
    field_number = zlib.crc32(bytearray(variable_name, 'utf8'))
    # Reduce to the correct amount of values
    field_number = (field_number % (536870911 - 1000))
    # Account for the fact that we must not use 0
    field_number += 1
    # Account for the fact that we must not use 19000 to 19999
    if field_number >= 19000:
        field_number += 1000

    return field_number


def to_proto_import(namespaced_type):
    assert isinstance(namespaced_type, rosidl.NamespacedType)
    return '/'.join(namespaced_type.namespaces + [convert_camel_case_to_lower_case_underscore(namespaced_type.name)]) + '.proto'


def collect_proto_imports(rosidl_message):
    assert isinstance(rosidl_message, rosidl.Message)
    import_set = set()

    for member in rosidl_message.structure.members:
        if isinstance(member.type, rosidl.NamespacedType):
            namespaced_type = member.type
        elif isinstance(member.type, rosidl.AbstractNestedType) \
                and isinstance(member.type.value_type, rosidl.NamespacedType):
            namespaced_type = member.type.value_type
        else:
            continue

        import_set.add(to_proto_import(namespaced_type))

    return import_set


def generate_proto(generator_arguments_file):
    mapping = {
        'idl.proto.em': '%s.proto',
    }
    generate_files(
        generator_arguments_file,
        mapping,
        # Don't keep case - we want snake case
        keep_case=False
    )
    return 0


def compile_proto(protoc_path, proto_path_list, cpp_out_dir, proto_files, package_name):
    protoc_cmd = [protoc_path]

    for path in proto_path_list:
        protoc_cmd.append('--proto_path=' + path)

    protoc_cmd.append(
        f'--cpp_out=dllexport_decl=ROSIDL_ADAPTER_PROTO_PUBLIC__{package_name}:{cpp_out_dir}')
    protoc_cmd = protoc_cmd + proto_files

    subprocess.check_call(protoc_cmd)
