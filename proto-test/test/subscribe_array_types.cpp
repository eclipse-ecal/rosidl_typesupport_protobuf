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
#include "test_msgs/msg/arrays__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/bounded_plain_sequences__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/bounded_sequences__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/multi_nested__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/nested__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/unbounded_sequences__typeadapter_protobuf_cpp.hpp"


#include "subscribe_array_types.hpp"
#include "subscribe_helper.hpp"

rclcpp::SubscriptionBase::SharedPtr subscribe_arrays(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Arrays>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::ArraysTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_unbounded_sequences(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::UnboundedSequences>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::UnboundedSequencesTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_bounded_plain_sequences(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::BoundedPlainSequences>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::BoundedPlainSequencesTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_bounded_sequences(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::BoundedSequences>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::BoundedSequencesTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_multi_nested(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::MultiNested>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::MultiNestedTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_nested(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Nested>> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::NestedTypeAdapter>(
    node, message_type, expected_messages, received_messages);
}
