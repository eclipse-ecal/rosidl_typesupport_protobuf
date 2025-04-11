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

import pathlib

import pytest

from rosidl_adapter_proto import collect_proto_imports
from rosidl_adapter_proto import compute_proto_field_number
from rosidl_adapter_proto import generate_proto
from rosidl_adapter_proto import MSG_TYPE_TO_PROTO
from rosidl_adapter_proto import to_proto_import
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Message
from rosidl_parser.parser import parse_idl_file


MESSAGE_IDL_LOCATOR = IdlLocator(pathlib.Path(__file__).parent,
                                 pathlib.Path('msg') / 'BoolTest.idl')


@pytest.fixture(scope='module')
def message_idl_file():
    return parse_idl_file(MESSAGE_IDL_LOCATOR)


def search_word(file_path, word):
    with open(file_path, 'r') as file:
        content = file.read()
        if word in content:
            return True
        return False


def get_member_name(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    member = messages[0].structure.members[0]
    return member.name


def test_message_proto_generated_invalid_argument():
    with pytest.raises(Exception):
        generate_file_argument = IdlLocator(
            pathlib.Path(__file__).parent,
            pathlib.Path('msg') / 'empty_document.json')
        rc = generate_proto(generate_file_argument.get_absolute_path())

        assert rc is None


def test_message_proto_generated_empty_file():
    with pytest.raises(Exception):
        generate_file_argument = IdlLocator(
            pathlib.Path(__file__).parent,
            pathlib.Path('msg') / 'test_rosidl_adapter__empty_args.json')
        rc = generate_proto(generate_file_argument.get_absolute_path())

        assert rc is None


def test_message_proto_generated(message_idl_file):
    generate_file_argument = IdlLocator(
        pathlib.Path(__file__).parent,
        pathlib.Path('msg') / 'test_rosid_adapter_proto__arguments.json')
    rc = generate_proto(generate_file_argument.get_absolute_path())

    assert rc is not None

    messages = message_idl_file.content.get_elements_of_type(Message)
    member = messages[0].structure.members[0]
    field_number = compute_proto_field_number(member.name)
    proto_type = MSG_TYPE_TO_PROTO[member.type.typename]

    proto_file_name = IdlLocator(
        pathlib.Path(__file__).parent,
        pathlib.Path('msg') / 'bool_test.proto')

    assert search_word(proto_file_name.get_absolute_path(), member.name) is True
    assert search_word(proto_file_name.get_absolute_path(), str(field_number)) is True
    assert search_word(proto_file_name.get_absolute_path(), proto_type) is True


def test_compute_proto_field_number_is_greter_than_0(message_idl_file):
    member_name = get_member_name(message_idl_file)
    field_number = compute_proto_field_number(member_name)

    assert field_number > 0


def test_compute_proto_field_number_is_not_in_the_reserved_range(message_idl_file):
    member_name = get_member_name(message_idl_file)
    field_number = compute_proto_field_number(member_name)

    assert field_number not in range(19000, 19999)


def test_compute_proto_field_number_repeated_with_same_member_name(message_idl_file):
    member_name = get_member_name(message_idl_file)
    field_number = compute_proto_field_number(member_name)
    second_field_number = compute_proto_field_number('data')

    assert field_number == second_field_number


def test_msg_type_to_proto_from_message_file(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    member = messages[0].structure.members[0]
    proto_type = MSG_TYPE_TO_PROTO[member.type.typename]
    assert proto_type == 'bool'


def test_msg_type_to_proto_mapping():
    idl_messages = ['boolean', 'octet', 'char', 'wchar', 'float',
                    'double', 'long double', 'uint8', 'int8',
                    'uint16', 'int16', 'uint32', 'int32', 'uint64',
                    'int64', 'string', 'wstring']
    expected_proto_type_ = ['bool', 'uint32', 'uint32', 'uint32',
                            'float', 'double', 'double', 'uint32',
                            'int32', 'uint32', 'int32', 'fixed32',
                            'sfixed32', 'fixed64', 'sfixed64',
                            'string', 'bytes']
    index = 0
    for idl_message in idl_messages:
        proto_type = MSG_TYPE_TO_PROTO[idl_message]
        assert expected_proto_type_[index] == proto_type
        index += 1


def test_msg_type_to_proto_invalid():
    with pytest.raises(Exception):
        MSG_TYPE_TO_PROTO['integer']


def test_to_proto_import(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    namespace_type = messages[0].structure.namespaced_type
    proto_import = to_proto_import(namespace_type)
    assert proto_import == 'rosidl_adapter_proto/bool.proto'


def test_to_proto_import_invalid_argument():
    with pytest.raises(Exception):
        to_proto_import('invalid argument')


def test_collect_proto_import(message_idl_file):
    messages = message_idl_file.content.get_elements_of_type(Message)
    proto_import_set = set()
    for message in messages:
        proto_import_set.update(collect_proto_imports(message))
    for proto_file in proto_import_set:
        assert proto_file == 'rosidl_adapter_proto/bool.proto'


def test_collect_proto_import_invalid_argument():
    with pytest.raises(Exception):
        collect_proto_imports('string value')
