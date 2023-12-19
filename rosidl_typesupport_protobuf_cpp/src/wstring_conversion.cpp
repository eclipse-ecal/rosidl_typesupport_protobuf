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

#include "rosidl_typesupport_protobuf_cpp/wstring_conversion.hpp"

#include <cstring>

namespace rosidl_typesupport_protobuf_cpp
{

void write_to_string(const std::u16string & u16str, std::string & str)
{
  auto data_size = u16str.size() * sizeof(std::u16string::value_type);
  str.resize(data_size);
  auto str_start = &str[0];
  std::memcpy(str_start, u16str.data(), data_size);
}

void write_to_u16string(const std::string & str, std::u16string & u16str)
{
  u16str.resize(str.size() / sizeof(std::u16string::value_type));
  auto wstr_start = &u16str[0];
  std::memcpy(wstr_start, str.data(), str.size());
}

}  // namespace rosidl_typesupport_protobuf_cpp
