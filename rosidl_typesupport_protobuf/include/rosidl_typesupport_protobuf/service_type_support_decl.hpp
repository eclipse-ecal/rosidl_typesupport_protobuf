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

#include <rosidl_typesupport_protobuf/visibility_control.h>

// rosidl_service_type_support_t
#include "rosidl_typesupport_protobuf/rosidl_generator_c_pkg_adapter.hpp"

namespace rosidl_typesupport_protobuf
{

/// Get the rosidl service typesupport handler of the type.
/**
 * This is implemented in the shared library provided by this package.a
 * \return The rosidl_service_type_support_t of type T.
 */
template<typename T>
ROSIDL_TYPESUPPORT_PROTOBUF_PUBLIC
const rosidl_service_type_support_t * get_service_type_support_handle();

}  // namespace rosidl_typesupport_protobuf
