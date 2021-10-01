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

import os

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_parser import definition

# A postfix for the protobuf package name / the c++ namespace
PROTO_PACKAGE_POSTFIX = "pb"

DISTROS = {"dashing": 0, "eloquent": 1, "foxy": 2, "galactic": 3}
CURRENT_DISTRO_NAME = os.getenv("ROS_DISTRO")
CURRENT_DISTRO = DISTROS[CURRENT_DISTRO_NAME]

_TYPE_SUPPORT_NAME = ""
_NAMESPACE_DELIMETER = ""

def set_type_support_name(val):
  global _TYPE_SUPPORT_NAME
  _TYPE_SUPPORT_NAME = val

def set_namespace_delimeter(val):
  global _NAMESPACE_DELIMETER
  _NAMESPACE_DELIMETER = val

def ros_message_header(package_name, interface_path):
  include_parts = [package_name] + list(interface_path.parents[0].parts)
  if CURRENT_DISTRO >= DISTROS["foxy"]:
    include_parts += ["detail"] 
  include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
  include_base = '/'.join(include_parts)

  return f"{include_base}__struct.hpp"

def ros_message_header_c(package_name, interface_path):
  include_parts = [package_name] + list(interface_path.parents[0].parts)
  if CURRENT_DISTRO >= DISTROS["foxy"]:
    include_parts += ["detail"]
  include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
  include_base = '/'.join(include_parts)

  return f"{include_base}__struct.h"

def ros_message_functions_header_c(package_name, interface_path):
  include_parts = [package_name] + list(interface_path.parents[0].parts)
  if CURRENT_DISTRO >= DISTROS["foxy"]:
     include_parts += ["detail"]
  include_parts += [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
  include_base = '/'.join(include_parts)

  return f"{include_base}__functions.h"

def ros_message_functions_header_c_from_namespace(namespace, name):
  include_parts = list(namespace)
  if CURRENT_DISTRO >= DISTROS["foxy"]:
     include_parts += ["detail"]
  include_parts += [convert_camel_case_to_lower_case_underscore(name)]
  include_base = '/'.join(include_parts)

  return f"{include_base}__functions.h"

def protobuf_message_header(package_name, interface_path):
  include_parts = [package_name] + list(interface_path.parents[0].parts)
  include_base = '/'.join(include_parts)
  include_prefix = interface_path.stem

  return '/'.join(include_parts + [include_prefix + ".pb.h"])

def typesupport_header(package_name, interface_path):
  include_parts = [package_name] + list(interface_path.parents[0].parts) + [convert_camel_case_to_lower_case_underscore(interface_path.stem)]
  include_base = '/'.join(include_parts)
  
  return f"{include_base}__{_TYPE_SUPPORT_NAME}.hpp"

def visibility_control_header(package_name):
  return f"{package_name}/{_TYPE_SUPPORT_NAME}__visibility_control.h"

def ros_type_namespace(package_name, interface_path):
  return _NAMESPACE_DELIMETER.join([package_name] + list(interface_path.parents[0].parts))

def ros_type_name(message):
  return message.structure.namespaced_type.name

def ros_type(package_name, interface_path, message):
  ros_type_ns = ros_type_namespace(package_name, interface_path)
  ros_type_nm = ros_type_name(message)
  return "::" + _NAMESPACE_DELIMETER.join([ros_type_ns, ros_type_nm])

def ros_type_from_namespaced_type(namespaced_type):
  return "::" + _NAMESPACE_DELIMETER.join(namespaced_type.namespaces + [namespaced_type.name])

def ros_type_from_namespaced_type_c(namespaced_type):
  return "::" + _NAMESPACE_DELIMETER.join(namespaced_type.namespaces + [namespaced_type.name])

def ros_service_namespace(package_name, interface_path):
  return _NAMESPACE_DELIMETER.join([package_name] + list(interface_path.parents[0].parts))

def ros_service_name(service):
  return service.namespaced_type.name

def ros_service_type(package_name, interface_path, service):
  ros_type_ns = ros_service_namespace(package_name, interface_path)
  ros_type_nm = ros_service_name(service)
  return "::" + _NAMESPACE_DELIMETER.join([ros_type_ns, ros_type_nm])

def protobuf_type(package_name, interface_path, message):
  namespace = "::".join([package_name] + list(interface_path.parents[0].parts))
  return "::" + "::".join([namespace, PROTO_PACKAGE_POSTFIX, ros_type_name(message)])

def protobuf_type_from_namespaced_type(namespaced_type):
  return "::" + "::".join(namespaced_type.namespaces + [PROTO_PACKAGE_POSTFIX, namespaced_type.name])

def protobuf_type_from_namespaced_type_c(namespaced_type):
  return "::" + "::".join(namespaced_type.namespaces + [PROTO_PACKAGE_POSTFIX, namespaced_type.name])
