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
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/rosidl_adapter_proto__visibility_control.h"
#include "builtin_interfaces/rosidl_adapter_proto__visibility_control.h"
#include "test_msgs/msg/basic_types__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/builtins__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/constants__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/defaults__typeadapter_protobuf_cpp.hpp"
#include "test_msgs/msg/empty__typeadapter_protobuf_cpp.hpp"

#include "subscribe_basic_types.hpp"
#include "subscribe_helper.hpp"

rclcpp::SubscriptionBase::SharedPtr subscribe_empty(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Empty>> & messages_expected,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::EmptyTypeAdapter>(
    node, message_type, messages_expected, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_basic_types(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::BasicTypes>> & messages_expected,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::BasicTypesTypeAdapter>(
    node, message_type, messages_expected, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_builtins(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Builtins>> & messages_expected,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::BuiltinsTypeAdapter>(
    node, message_type, messages_expected, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_constants(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Constants>> & messages_expected,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::ConstantsTypeAdapter>(
    node, message_type, messages_expected, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_defaults(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Defaults>> & messages_expected,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::typesupport_protobuf_cpp::DefaultsTypeAdapter>(
    node, message_type, messages_expected, received_messages);
}
