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
from rosidl_cmake import convert_camel_case_to_lower_case_underscore
import rosidl_parser.definition as rosidl

from rosidl_adapter_proto import PROTO_PACKAGE_POSTFIX
from rosidl_adapter_proto import MSG_TYPE_TO_PROTO
from rosidl_adapter_proto import PROTO_ACTION_SEND_GOAL_CALL_NAME
from rosidl_adapter_proto import PROTO_ACTION_GET_RESULT_CALL_NAME

}@
option cc_generic_services = true;

@#
@# ================ Message definitions ================
@#
@[for message in [action.goal,
                  action.send_goal_service.request_message,
                  action.send_goal_service.response_message,
                  action.result,
                  action.get_result_service.request_message,
                  action.get_result_service.response_message,
                  action.feedback,
                  action.feedback_message,
                 ]
]@
@{
TEMPLATE(
    'msg.proto.em',
    package_name=package_name,
    interface_path=interface_path,
    message=message,
)
}@


@[end for]@
@#
@# ================ Service definitions ================
@#
service @(action.namespaced_type.name)
{
  rpc @(PROTO_ACTION_SEND_GOAL_CALL_NAME) (@(action.send_goal_service.request_message.structure.namespaced_type.name)) returns (@(action.send_goal_service.response_message.structure.namespaced_type.name));
  rpc @(PROTO_ACTION_GET_RESULT_CALL_NAME) (@(action.get_result_service.request_message.structure.namespaced_type.name)) returns (@(action.get_result_service.response_message.structure.namespaced_type.name));
}