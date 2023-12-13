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

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/rosidl_adapter_proto__visibility_control.h"
#include "builtin_interfaces/rosidl_adapter_proto__visibility_control.h"
#include "test_msgs/msg/strings__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/w_strings__typeadapter_protobuf_cpp.hpp"


#include "subscribe_helper.hpp"
#include "subscribe_string_types.hpp"

rclcpp::SubscriptionBase::SharedPtr subscribe_strings(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Strings>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::StringsTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_wstrings(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::WStrings>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::WStringsTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}
