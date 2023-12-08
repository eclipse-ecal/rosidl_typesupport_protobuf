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

namespace rosidl_typesupport_protobuf
{

typedef struct message_type_support_t
{
  const char * message_namespace;
  const char * message_name;

  bool (* serialize)(const void * ros_message, std::string & serialized_message);
  bool (* deserialize)(void * ros_message, const void * serialized_data, size_t size);
  std::string (* get_descriptor)();
} message_type_support_t;

}  // namespace rosidl_typesupport_protobuf
