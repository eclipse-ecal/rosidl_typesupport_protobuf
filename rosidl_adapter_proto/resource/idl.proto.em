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
from rosidl_adapter_proto import PROTO_PACKAGE_POSTFIX
}@
// generated from rosidl_adapter_proto/resource/idl.proto.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice

syntax = "proto3";

package @('.'.join([package_name] + list(interface_path.parts)[:-1] + [PROTO_PACKAGE_POSTFIX]));

@{
#######################################################################
# Protobuf imports
#######################################################################
from rosidl_adapter_proto import collect_proto_imports
from rosidl_adapter_proto import to_proto_import
import rosidl_parser.definition as rosidl

proto_import_set = set()
already_imported = set()

# Collect imports from plain messages
for message in content.get_elements_of_type(rosidl.Message):
    proto_import_set |= collect_proto_imports(message)

    already_imported.add(to_proto_import(message.structure.namespaced_type))

# Collect imports from services
for service in content.get_elements_of_type(rosidl.Service):
    proto_import_set |= collect_proto_imports(service.request_message)
    proto_import_set |= collect_proto_imports(service.response_message)

    already_imported.add(to_proto_import(service.namespaced_type))
    already_imported.add(to_proto_import(service.request_message.structure.namespaced_type))
    already_imported.add(to_proto_import(service.response_message.structure.namespaced_type))

# Collect imports from actions
for action in content.get_elements_of_type(rosidl.Action):
    proto_import_set |= collect_proto_imports(action.goal)
    proto_import_set |= collect_proto_imports(action.send_goal_service.request_message)
    proto_import_set |= collect_proto_imports(action.send_goal_service.response_message)
    proto_import_set |= collect_proto_imports(action.result)
    proto_import_set |= collect_proto_imports(action.get_result_service.request_message)
    proto_import_set |= collect_proto_imports(action.get_result_service.response_message)
    proto_import_set |= collect_proto_imports(action.feedback)
    proto_import_set |= collect_proto_imports(action.feedback_message)

    already_imported.add(to_proto_import(action.namespaced_type))
    already_imported.add(to_proto_import(action.goal.structure.namespaced_type))
    already_imported.add(to_proto_import(action.send_goal_service.request_message.structure.namespaced_type))
    already_imported.add(to_proto_import(action.send_goal_service.response_message.structure.namespaced_type))
    already_imported.add(to_proto_import(action.result.structure.namespaced_type))
    already_imported.add(to_proto_import(action.get_result_service.request_message.structure.namespaced_type))
    already_imported.add(to_proto_import(action.get_result_service.response_message.structure.namespaced_type))
    already_imported.add(to_proto_import(action.feedback.structure.namespaced_type))
    already_imported.add(to_proto_import(action.feedback_message.structure.namespaced_type))

for already_imported_proto_file in already_imported:
    if already_imported_proto_file in proto_import_set:
        proto_import_set.remove(already_imported_proto_file)
}@
@[for import_proto_file_path in sorted(proto_import_set)]@
import "@(import_proto_file_path)";
@[end for]@
@#
@# Newline after import statements
@#
@[if len(proto_import_set) != 0]@

@[end if]@
@{
#######################################################################
# Handle message
#######################################################################
from rosidl_parser.definition import Message
for message in content.get_elements_of_type(Message):
    TEMPLATE(
        'msg.proto.em',
        package_name=package_name,
        interface_path=interface_path,
        message=message,
    )

########################################################################
## Handle service
########################################################################
from rosidl_parser.definition import Service
for service in content.get_elements_of_type(Service):
    TEMPLATE(
        'srv.proto.em',
        package_name=package_name,
        interface_path=interface_path,
        service=service,
    )

########################################################################
## Handle action
########################################################################
from rosidl_parser.definition import Action
for action in content.get_elements_of_type(Action):
    TEMPLATE(
        'action.proto.em',
        package_name=package_name,
        interface_path=interface_path,
        action=action,
    )
}@
