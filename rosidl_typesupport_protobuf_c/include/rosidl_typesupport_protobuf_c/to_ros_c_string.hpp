/* ================================ Apache 2.0 =================================
 *
 * Copyright (C) 2021 Continental
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ================================ Apache 2.0 =================================
 */

#pragma once

#include <string>

#include "rosidl_typesupport_protobuf/visibility_control.h"
#include "rosidl_typesupport_protobuf/rosidl_generator_c_pkg_adapter.hpp"

namespace typesupport_protobuf_c
{

ROSIDL_TYPESUPPORT_PROTOBUF_PUBLIC
void to_ros_c_string(const std::string & str, rosidl_runtime_c__String & ros_str);

}
