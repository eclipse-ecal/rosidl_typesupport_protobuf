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

#include <string.h>

#include "gtest/gtest.h"
#include <iostream>

#include "rosidl_typesupport_protobuf_cpp/wstring_conversion.hpp"

using rosidl_typesupport_protobuf_cpp::write_to_string;
using rosidl_typesupport_protobuf_cpp::write_to_u16string;

TEST(test_wstring_conversion, u16string_to_string) {
  std::string actual;

  // Default string
  write_to_string(std::u16string(), actual);
  EXPECT_EQ(std::string(), actual);

  // Empty string
  write_to_string(std::u16string(u""), actual);
  EXPECT_EQ(std::string(""), actual);

  // Non-empty string
  write_to_string(std::u16string(u"Hello World"), actual);
  const char byteArray[] = {'H', 0x00, 'e', 0x00, 'l', 0x00, 'l', 0x00,
                            'o', 0x00, ' ', 0x00, 'W', 0x00, 'o', 0x00,
                            'r', 0x00, 'l', 0x00, 'd', 0x00};
  std::string expected(byteArray, sizeof(byteArray));
  EXPECT_EQ(expected, actual);
}

TEST(test_wstring_conversion, string_to_u16string) {
  std::u16string actual;

  // Default string
  write_to_u16string(std::string(), actual);
  EXPECT_EQ(std::u16string(), actual);

  // Empty string
  write_to_u16string(std::string(""), actual);
  EXPECT_EQ(std::u16string(u""), actual);

  // Non-empty string
  const char byteArray_input[] = {'H', 0x00, 'e', 0x00, 'l', 0x00, 'l', 0x00,
                                  'o', 0x00, ' ', 0x00, 'W', 0x00, 'o', 0x00,
                                  'r', 0x00, 'l', 0x00, 'd', 0x00};
  std::string input_string(byteArray_input, sizeof(byteArray_input));
  write_to_u16string(input_string, actual);
  std::u16string expected(u"Hello World");
  EXPECT_EQ(expected, actual);
}