// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSIDL_TYPEADAPTER_PROTOBUF_TEST__TYPEADAPT_MESSAGE_FIXTURES_HPP_
#define ROSIDL_TYPEADAPTER_PROTOBUF_TEST__TYPEADAPT_MESSAGE_FIXTURES_HPP_

#include <cassert>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <string>

#include "test_msgs/rosidl_adapter_proto__visibility_control.h"
#include "builtin_interfaces/rosidl_adapter_proto__visibility_control.h"
#include "test_msgs/msg/arrays__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/basic_types__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/bounded_plain_sequences__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/bounded_sequences__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/builtins__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/constants__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/defaults__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/empty__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/multi_nested__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/nested__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/strings__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/unbounded_sequences__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/w_strings__typeadapter_protobuf_cpp.hpp"

static inline std::string
from_u8string(const std::string & s)
{
  return s;
}

static inline std::string
from_u8string(std::string && s)
{
  return std::move(s);
}

#if defined(__cpp_lib_char8_t)
static inline std::string
from_u8string(const std::u8string & s)
{
  return std::string(s.begin(), s.end());
}
#endif

typedef std::shared_ptr<test_msgs::msg::pb::Empty> EmptySharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::BasicTypes> BasicTypesSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::Constants> ConstantsSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::Defaults> DefaultsSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::Strings> StringsSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::Arrays> ArraysSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::UnboundedSequences> UnboundedSequencesSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::BoundedPlainSequences> BoundedPlainSequencesSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::BoundedSequences> BoundedSequencesSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::MultiNested> MultiNestedSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::Nested> NestedSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::Builtins> BuiltinsSharedPtr;
typedef std::shared_ptr<test_msgs::msg::pb::WStrings> WStringsSharedPtr;

static inline std::vector<EmptySharedPtr>
get_proto_messages_empty()
{
  std::vector<EmptySharedPtr> messages;
  auto msg = std::make_shared<test_msgs::msg::pb::Empty>();
  messages.push_back(msg);
  return messages;
}

static inline std::vector<BasicTypesSharedPtr>
get_proto_messages_basic_types()
{
  std::vector<BasicTypesSharedPtr> messages;
  // {
  //   auto msg = std::make_shared<test_msgs::msg::pb::BasicTypes>();
  //   msg->set_bool_value(false);
  //   msg->set_byte_value(0);
  //   msg->set_char_value(0);
  //   msg->set_float32_value(0.0f);
  //   msg->set_float64_value(0);
  //   msg->set_int8_value(0);
  //   msg->set_uint8_value(0);
  //   msg->set_int16_value(0);
  //   msg->set_uint16_value(0);
  //   msg->set_int32_value(0);
  //   msg->set_uint32_value(0);
  //   msg->set_int64_value(0);
  //   msg->set_uint64_value(0);
  //   messages.push_back(msg);
  // }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BasicTypes>();
    msg->set_bool_value(true);
    msg->set_byte_value(255);
    msg->set_char_value(255);
    msg->set_float32_value(1.125f);
    msg->set_float64_value(1.125);
    msg->set_int8_value(std::numeric_limits<int8_t>::max());
    msg->set_uint8_value(std::numeric_limits<uint8_t>::max());
    msg->set_int16_value(std::numeric_limits<int16_t>::max());
    msg->set_uint16_value(std::numeric_limits<uint16_t>::max());
    msg->set_int32_value(std::numeric_limits<int32_t>::max());
    msg->set_uint32_value(std::numeric_limits<uint32_t>::max());
    msg->set_int64_value(std::numeric_limits<int64_t>::max());
    msg->set_uint64_value(std::numeric_limits<uint64_t>::max());
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BasicTypes>();
    msg->set_bool_value(false);
    msg->set_byte_value(0);
    msg->set_char_value(0);
    msg->set_float32_value(-2.125f);
    msg->set_float64_value(-2.125);
    msg->set_int8_value(std::numeric_limits<int8_t>::min());
    msg->set_uint8_value(0);
    msg->set_int16_value(std::numeric_limits<int16_t>::min());
    msg->set_uint16_value(0);
    msg->set_int32_value(std::numeric_limits<int32_t>::min());
    msg->set_uint32_value(0);
    msg->set_int64_value(std::numeric_limits<int64_t>::min());
    msg->set_uint64_value(0);
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BasicTypes>();
    msg->set_bool_value(true);
    msg->set_byte_value(1);
    msg->set_char_value(1);
    msg->set_float32_value(1.0f);
    msg->set_float64_value(1);
    msg->set_int8_value(1);
    msg->set_uint8_value(1);
    msg->set_int16_value(1);
    msg->set_uint16_value(1);
    msg->set_int32_value(1);
    msg->set_uint32_value(1);
    msg->set_int64_value(1);
    msg->set_uint64_value(1);
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<ConstantsSharedPtr>
get_proto_messages_constants()
{
  std::vector<ConstantsSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Constants>();
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<DefaultsSharedPtr>
get_proto_messages_defaults()
{
  std::vector<DefaultsSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Defaults>();
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<StringsSharedPtr>
get_proto_messages_strings()
{
  std::vector<StringsSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Strings>();
    msg->set_string_value("");
    msg->set_bounded_string_value("");
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Strings>();
    msg->set_string_value("Hello world!");
    msg->set_bounded_string_value("Hello world!");
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Strings>();
    msg->set_string_value(from_u8string(u8"Hell\u00F6 W\u00F6rld!"));  // using umlaut
    msg->set_bounded_string_value(from_u8string(u8"Hell\u00F6 W\u00F6rld!"));  // using umlaut
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Strings>();
    msg->set_string_value("");
    msg->set_bounded_string_value("");
    for (size_t i = 0; i < 20000; ++i) {
      msg->set_string_value(msg->string_value() + std::to_string(i % 10));
    }
    for (size_t i = 0; i < 22; ++i) {
      msg->set_bounded_string_value(msg->bounded_string_value() + std::to_string(i % 10));
    }
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<ArraysSharedPtr>
get_proto_messages_arrays()
{
  auto basic_types_msgs = get_proto_messages_basic_types();
  std::vector<ArraysSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Arrays>();

    msg->add_bool_values(false);
    msg->add_bool_values(true);
    msg->add_bool_values(false);

    std::string values("000");
    values[0] = 0;
    values[1] = 255;
    values[2] = 0;
    msg->set_byte_values(values);

    values[0] = 0;
    values[1] = 255;
    values[2] = 0;
    msg->set_char_values(values);

    msg->add_float32_values(0.0f);
    msg->add_float32_values(1.125f);
    msg->add_float32_values(-2.125f);

    msg->add_float64_values(0);
    msg->add_float64_values(1.125);
    msg->add_float64_values(-2.125);

    values[0] = 0;
    values[1] = std::numeric_limits<int8_t>::max();
    values[2] = std::numeric_limits<int8_t>::min();
    msg->set_int8_values(values);

    values[0] = 0;
    values[1] = std::numeric_limits<uint8_t>::max();
    values[2] = 0;
    msg->set_uint8_values(values);

    msg->add_int16_values(0);
    msg->add_int16_values(std::numeric_limits<int16_t>::max());
    msg->add_int16_values(std::numeric_limits<int16_t>::min());

    msg->add_uint16_values(0);
    msg->add_uint16_values(std::numeric_limits<uint16_t>::max());
    msg->add_uint16_values(0);

    msg->add_int32_values(static_cast<int32_t>(0));
    msg->add_int32_values(std::numeric_limits<int32_t>::max());
    msg->add_int32_values(std::numeric_limits<int32_t>::min());

    msg->add_uint32_values(0);
    msg->add_uint32_values(std::numeric_limits<uint32_t>::max());
    msg->add_uint32_values(0);

    msg->add_int64_values(0);
    msg->add_int64_values(std::numeric_limits<int64_t>::max());
    msg->add_int64_values(std::numeric_limits<int64_t>::min());

    msg->add_uint64_values(0);
    msg->add_uint64_values(std::numeric_limits<uint64_t>::max());
    msg->add_uint64_values(0);

    msg->add_string_values("");
    msg->add_string_values("max value");
    msg->add_string_values("min value");

    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[0]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[1]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[2]);

    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<UnboundedSequencesSharedPtr>
get_proto_messages_unbounded_sequences()
{
  auto basic_types_msgs = get_proto_messages_basic_types();
  std::vector<UnboundedSequencesSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::UnboundedSequences>();
    msg->add_bool_values(true);
    msg->set_byte_values(std::string(1, 0xff));
    msg->set_char_values(std::string(1, 255));
    msg->add_float32_values(1.125f);
    msg->add_float64_values(1.125);
    msg->set_int8_values(std::string(1, std::numeric_limits<int8_t>::max()));
    msg->set_uint8_values(std::string(1, std::numeric_limits<uint8_t>::max()));
    msg->add_int16_values(std::numeric_limits<int16_t>::max());
    msg->add_uint16_values(std::numeric_limits<uint16_t>::max());
    msg->add_int32_values(std::numeric_limits<int32_t>::max());
    msg->add_uint32_values(std::numeric_limits<uint32_t>::max());
    msg->add_int64_values(std::numeric_limits<int64_t>::max());
    msg->add_uint64_values(std::numeric_limits<uint64_t>::max());
    msg->add_string_values("max value");
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[0]);
    msg->set_alignment_check(1);
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::UnboundedSequences>();
    msg->add_bool_values(false);
    msg->add_bool_values(true);

    std::string values;
    values[0] = 0;
    values[1] = 0xff;
    msg->set_byte_values(values);

    values[0] = 0;
    values[1] = 255;
    msg->set_char_values(values);

    msg->add_float32_values(0.0f);
    msg->add_float32_values(1.125f);
    msg->add_float32_values(-2.125f);

    msg->add_float64_values(0);
    msg->add_float64_values(1.125);
    msg->add_float64_values(-2.125);

    values[0] = 0;
    values[1] = std::numeric_limits<int8_t>::max();
    values[2] = std::numeric_limits<int8_t>::min();
    msg->set_int8_values(values);

    values[0] = 0;
    values[1] = std::numeric_limits<uint8_t>::max();
    msg->set_uint8_values(values);

    msg->add_int16_values(0);
    msg->add_int16_values(std::numeric_limits<int16_t>::max());
    msg->add_int16_values(std::numeric_limits<int16_t>::min());

    msg->add_uint16_values(0);
    msg->add_uint16_values(std::numeric_limits<uint16_t>::max());

    // The narrowing static cast is required to avoid build errors on Windows.
    msg->add_int32_values(static_cast<int32_t>(0));
    msg->add_int32_values(std::numeric_limits<int32_t>::max());
    msg->add_int32_values(std::numeric_limits<int32_t>::min());

    msg->add_uint32_values(0);
    msg->add_uint32_values(std::numeric_limits<uint32_t>::max());

    msg->add_int64_values(0);
    msg->add_int64_values(std::numeric_limits<int64_t>::max());
    msg->add_int64_values(std::numeric_limits<int64_t>::min());

    msg->add_uint64_values(0);
    msg->add_uint64_values(std::numeric_limits<uint64_t>::max());

    msg->add_string_values("");
    msg->add_string_values("max value");
    msg->add_string_values("optional min value");

    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[0]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[1]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[2]);
    msg->set_alignment_check(2);
    messages.push_back(msg);
  }
  // {
  //   auto msg = std::make_shared<test_msgs::msg::pb::UnboundedSequences>();
  //   // check sequences with more then 100 elements
  //   const size_t size = 1000;
  //   msg->set_bool_values.resize(size);
  //   msg->set_byte_values.resize(size);
  //   msg->set_char_values.resize(size);
  //   msg->set_float32_values.resize(size);
  //   msg->add_float64_values.resize(size);
  //   msg->set_int8_values.resize(size);
  //   msg->set_uint8_values.resize(size);
  //   msg->set_int16_values.resize(size);
  //   msg->set_uint16_values.resize(size);
  //   msg->set_int32_values.resize(size);
  //   msg->set_uint32_values.resize(size);
  //   msg->set_int64_values.resize(size);
  //   msg->set_uint64_values.resize(size);
  //   msg->set_string_values.resize(size);
  //   msg->set_basic_types_values.resize(size);
  //   for (size_t i = 0; i < size; ++i) {
  //     msg->set_bool_values[i] = (i % 2 != 0) ? true : false;
  //     msg->set_byte_values[i] = static_cast<uint8_t>(i);
  //     msg->set_char_values[i] = static_cast<char>(i % (1 << 8));
  //     msg->set_float32_values[i] = 1.125f * i;
  //     msg->add_float64_values[i] = 1.125 * i;
  //     msg->set_int8_values[i] = static_cast<int8_t>(i);
  //     msg->set_uint8_values[i] = static_cast<uint8_t>(i);
  //     msg->set_int16_values[i] = static_cast<int16_t>(i);
  //     msg->set_uint16_values[i] = static_cast<uint16_t>(i);
  //     msg->set_int32_values[i] = static_cast<int32_t>(i);
  //     msg->set_uint32_values[i] = static_cast<uint32_t>(i);
  //     msg->set_int64_values[i] = i;
  //     msg->set_uint64_values[i] = i;
  //     msg->set_string_values[i] = std::to_string(i);
  //     msg->set_basic_types_values[i] = *basic_types_msgs[i % basic_types_msgs.size()];
  //   }
  //   msg->set_alignment_check = 3;
  //   messages.push_back(msg);
  // }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::UnboundedSequences>();
    // check default sequences
    msg->set_alignment_check(4);
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<BoundedPlainSequencesSharedPtr>
get_proto_messages_bounded_plain_sequences()
{
  auto basic_types_msgs = get_proto_messages_basic_types();
  auto msg = std::make_shared<test_msgs::msg::pb::UnboundedSequences>();
  std::vector<BoundedPlainSequencesSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BoundedPlainSequences>();
    msg->add_bool_values(false);
    msg->add_bool_values(true);
    msg->add_bool_values(false);

    std::string values;
    values[0] = 0;
    values[1] = 1;
    values[2] = 0xff;
    msg->set_byte_values(values);

    values[0] = 0;
    values[1] = 1;
    values[2] = 255;
    msg->set_char_values(values);

    msg->add_float32_values(0.0f);
    msg->add_float32_values(1.125f);
    msg->add_float32_values(-2.125f);

    msg->add_float64_values(0);
    msg->add_float64_values(1.125);
    msg->add_float64_values(-2.125);

    values[0] = 0;
    values[1] = std::numeric_limits<int8_t>::max();
    values[2] = std::numeric_limits<int8_t>::min();
    msg->set_int8_values(values);

    values[0] = 0;
    values[1] = 1;
    values[2] = std::numeric_limits<uint8_t>::min();
    msg->set_uint8_values(values);

    msg->add_int16_values(0);
    msg->add_int16_values(std::numeric_limits<int16_t>::max());
    msg->add_int16_values(std::numeric_limits<int16_t>::min());

    msg->add_uint16_values(0);
    msg->add_uint16_values(1);
    msg->add_uint16_values(std::numeric_limits<uint16_t>::max());

    // The narrowing static cast is required to avoid build errors on Windows.
    msg->add_int32_values(static_cast<int32_t>(0));
    msg->add_int32_values(std::numeric_limits<int32_t>::max());
    msg->add_int32_values(std::numeric_limits<int32_t>::min());

    msg->add_uint32_values(0);
    msg->add_uint32_values(1);
    msg->add_uint32_values(std::numeric_limits<uint32_t>::max());

    msg->add_int64_values(0);
    msg->add_int64_values(std::numeric_limits<int64_t>::max());
    msg->add_int64_values(std::numeric_limits<int64_t>::min());

    msg->add_uint64_values(0);
    msg->add_uint64_values(1);
    msg->add_uint64_values(std::numeric_limits<uint64_t>::max());

    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[0]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[1]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[2]);
    msg->set_alignment_check(2);
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BoundedPlainSequences>();
    // check default sequences
    msg->set_alignment_check(4);
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<BoundedSequencesSharedPtr>
get_proto_messages_bounded_sequences()
{
  auto basic_types_msgs = get_proto_messages_basic_types();
  auto msg = std::make_shared<test_msgs::msg::pb::UnboundedSequences>();
  std::vector<BoundedSequencesSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BoundedSequences>();
    msg->add_bool_values(false);
    msg->add_bool_values(true);
    msg->add_bool_values(false);

    std::string values;
    values[0] = 0;
    values[1] = 1;
    values[2] = 0xff;
    msg->set_byte_values(values);

    values[0] = 0;
    values[1] = 1;
    values[2] = 255;
    msg->set_char_values(values);

    msg->add_float32_values(0.0f);
    msg->add_float32_values(1.125f);
    msg->add_float32_values(-2.125f);

    msg->add_float64_values(0);
    msg->add_float64_values(1.125);
    msg->add_float64_values(-2.125);

    values[0] = 0;
    values[1] = std::numeric_limits<int8_t>::max();
    values[2] = std::numeric_limits<int8_t>::min();
    msg->set_int8_values(values);

    values[0] = 0;
    values[1] = 1;
    values[2] = std::numeric_limits<uint8_t>::min();
    msg->set_uint8_values(values);

    msg->add_int16_values(0);
    msg->add_int16_values(std::numeric_limits<int16_t>::max());
    msg->add_int16_values(std::numeric_limits<int16_t>::min());

    msg->add_uint16_values(0);
    msg->add_uint16_values(1);
    msg->add_uint16_values(std::numeric_limits<uint16_t>::max());

    // The narrowing static cast is required to avoid build errors on Windows.
    msg->add_int32_values(static_cast<int32_t>(0));
    msg->add_int32_values(std::numeric_limits<int32_t>::max());
    msg->add_int32_values(std::numeric_limits<int32_t>::min());

    msg->add_uint32_values(0);
    msg->add_uint32_values(1);
    msg->add_uint32_values(std::numeric_limits<uint32_t>::max());

    msg->add_int64_values(0);
    msg->add_int64_values(std::numeric_limits<int64_t>::max());
    msg->add_int64_values(std::numeric_limits<int64_t>::min());

    msg->add_uint64_values(0);
    msg->add_uint64_values(1);
    msg->add_uint64_values(std::numeric_limits<uint64_t>::max());

    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[0]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[1]);
    msg->add_basic_types_values()->CopyFrom(*basic_types_msgs[2]);
    msg->set_alignment_check(2);
    messages.push_back(msg);
  }
  {
    auto msg = std::make_shared<test_msgs::msg::pb::BoundedSequences>();
    // check default sequences
    msg->set_alignment_check(4);
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<MultiNestedSharedPtr>
get_proto_messages_multi_nested()
{
  auto arrays_msgs = get_proto_messages_arrays();
  auto bounded_sequences_msgs = get_proto_messages_bounded_sequences();
  auto unbounded_sequences_msgs = get_proto_messages_unbounded_sequences();
  const std::size_t num_arrays = arrays_msgs.size();
  const std::size_t num_bounded_sequences = bounded_sequences_msgs.size();
  const std::size_t num_unbounded_sequences = unbounded_sequences_msgs.size();
  std::vector<MultiNestedSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::MultiNested>();
    for (std::size_t i = 0u; i < msg->array_of_arrays_size(); ++i) {
      msg->mutable_array_of_arrays(i)->CopyFrom(*arrays_msgs[i % num_arrays]);
    }
    for (std::size_t i = 0u; i < msg->array_of_bounded_sequences_size(); ++i) {
      msg->mutable_array_of_bounded_sequences(i)->CopyFrom(
        *bounded_sequences_msgs[i % num_bounded_sequences]);
    }
    for (std::size_t i = 0u; i < msg->array_of_unbounded_sequences_size(); ++i) {
      msg->mutable_array_of_unbounded_sequences(i)->CopyFrom(
        *unbounded_sequences_msgs[i % num_unbounded_sequences]);
    }
    const std::size_t sequence_size = 3u;
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->add_bounded_sequence_of_arrays()->CopyFrom(*arrays_msgs[i % num_arrays]);
    }
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->add_bounded_sequence_of_bounded_sequences()->CopyFrom(
        *bounded_sequences_msgs[i % num_bounded_sequences]);
    }
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->add_bounded_sequence_of_unbounded_sequences()->CopyFrom(
        *unbounded_sequences_msgs[i % num_unbounded_sequences]);
    }
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->add_unbounded_sequence_of_arrays()->CopyFrom(*arrays_msgs[i % num_arrays]);
    }
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->add_unbounded_sequence_of_bounded_sequences()->CopyFrom(
        *bounded_sequences_msgs[i % num_bounded_sequences]);
    }
    for (std::size_t i = 0u; i < sequence_size; ++i) {
      msg->add_unbounded_sequence_of_unbounded_sequences()->CopyFrom(
        *unbounded_sequences_msgs[i % num_unbounded_sequences]);
    }
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<NestedSharedPtr>
get_proto_messages_nested()
{
  std::vector<NestedSharedPtr> messages;
  auto basic_types_msgs = get_proto_messages_basic_types();
  for (auto basic_types_msg : basic_types_msgs) {
    auto msg = std::make_shared<test_msgs::msg::pb::Nested>();
    msg->mutable_basic_types_value()->CopyFrom(*basic_types_msg);
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<BuiltinsSharedPtr>
get_proto_messages_builtins()
{
  std::vector<BuiltinsSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::Builtins>();
    msg->mutable_duration_value()->set_sec(-1234567890);
    msg->mutable_duration_value()->set_nanosec(123456789);
    msg->mutable_time_value()->set_sec(-1234567890);
    msg->mutable_time_value()->set_nanosec(987654321);
    messages.push_back(msg);
  }
  return messages;
}

static inline std::vector<WStringsSharedPtr>
get_proto_messages_wstrings()
{
  std::vector<WStringsSharedPtr> messages;
  {
    auto msg = std::make_shared<test_msgs::msg::pb::WStrings>();
    msg->add_array_of_wstrings("");
    msg->add_array_of_wstrings("");
    msg->add_array_of_wstrings("");
    // msg->set_wstring_value("");
    // msg->set_array_of_wstrings(0, u"1");
    // msg->set_array_of_wstrings[1] = u"two";
    // msg->set_array_of_wstrings[2] = u"三";  // "One" in Japanese
    // msg->set_bounded_sequence_of_wstrings.resize(2);
    // msg->set_bounded_sequence_of_wstrings[0] = u"one";
    // msg->set_bounded_sequence_of_wstrings[1] = u"二";  // "Two" in Japanese
    // msg->set_unbounded_sequence_of_wstrings.resize(4);
    // msg->set_unbounded_sequence_of_wstrings[0] = u".";
    // msg->set_unbounded_sequence_of_wstrings[1] = u"..";
    // msg->set_unbounded_sequence_of_wstrings[2] = u"...";
    // msg->set_unbounded_sequence_of_wstrings[3] = u"四";  // "Four" in Japanese
    messages.push_back(msg);
  }
  // {
  //   auto msg = std::make_shared<test_msgs::msg::pb::WStrings>();
  //   msg->set_wstring_value(u"ascii";
  //   messages.push_back(msg);
  // }
  // {
  //   auto msg = std::make_shared<test_msgs::msg::pb::WStrings>();
  //   msg->set_wstring_value(u"Hell\u00F6 W\u00F6rld!";  // using umlaut
  //   messages.push_back(msg);
  // }
  // {
  //   auto msg = std::make_shared<test_msgs::msg::pb::WStrings>();
  //   msg->set_wstring_value(u"ハローワールド";  // "Hello world" in Japanese
  //    messages.push_back(msg);
  // }
  return messages;
}

#endif  // ROSIDL_TYPEADAPTER_PROTOBUF_TEST__TYPEADAPT_MESSAGE_FIXTURES_HPP_
