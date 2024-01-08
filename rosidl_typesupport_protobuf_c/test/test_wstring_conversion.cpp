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

#include "rosidl_runtime_c/u16string_functions.h"

#include "rosidl_typesupport_protobuf_c/wstring_conversion.hpp"

using rosidl_typesupport_protobuf_c::write_to_string;
using rosidl_typesupport_protobuf_c::write_to_u16string;

TEST(test_wstring_conversion, u16string_to_string) {
  std::string actual;
  rosidl_runtime_c__U16String input;
  ASSERT_TRUE(rosidl_runtime_c__U16String__init(&input));

  // Default string
  write_to_string(input, actual);
  EXPECT_EQ(std::string(), actual);

  // Empty String
  ASSERT_TRUE(
      rosidl_runtime_c__U16String__assign(&input, (const uint16_t *)u""));
  write_to_string(input, actual);
  EXPECT_EQ(std::string(""), actual);

  // Non-empty string
  ASSERT_TRUE(rosidl_runtime_c__U16String__assign(
      &input, (const uint16_t *)u"Hello World"));
  write_to_string(input, actual);
  const char byteArray[] = {'H', 0x00, 'e', 0x00, 'l', 0x00, 'l', 0x00,
                            'o', 0x00, ' ', 0x00, 'W', 0x00, 'o', 0x00,
                            'r', 0x00, 'l', 0x00, 'd', 0x00};
  std::string expected(byteArray, sizeof(byteArray));
  EXPECT_EQ(expected, actual);
}

TEST(test_wstring_conversion, string_to_u16string) {
  rosidl_runtime_c__U16String actual;
  ASSERT_TRUE(rosidl_runtime_c__U16String__init(&actual));

  // Default string
  write_to_u16string(std::string(), actual);
  EXPECT_EQ(0, memcmp(u"", actual.data, actual.size));

  // Empty String
  write_to_u16string(std::string(""), actual);
  EXPECT_EQ(0, memcmp(u"", actual.data, actual.size));

  // Non-empty string
  const char byteArray_input[] = {'H', 0x00, 'e', 0x00, 'l', 0x00, 'l', 0x00,
                                  'o', 0x00, ' ', 0x00, 'W', 0x00, 'o', 0x00,
                                  'r', 0x00, 'l', 0x00, 'd', 0x00};
  std::string input_string(byteArray_input, sizeof(byteArray_input));
  write_to_u16string(input_string, actual);
  EXPECT_EQ(0, memcmp(u"Hello World", actual.data, actual.size));
}