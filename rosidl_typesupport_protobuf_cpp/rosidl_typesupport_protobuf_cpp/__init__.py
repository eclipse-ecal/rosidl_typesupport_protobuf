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

from rosidl_cmake import generate_files


def generate_cpp(generator_arguments_file):
    mapping = {
        "idl__rosidl_typesupport_protobuf_cpp.hpp.em": "%s__rosidl_typesupport_protobuf_cpp.hpp",
        "idl__typeadapter_protobuf_cpp.hpp.em": "%s__typeadapter_protobuf_cpp.hpp",
        "idl__type_support.cpp.em": "detail/%s__type_support.cpp",
    }
    generate_files(generator_arguments_file, mapping)
    return 0
