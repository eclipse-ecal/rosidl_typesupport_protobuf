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

#ifndef SUBSCRIBE_STRING_TYPES_HPP_
#define SUBSCRIBE_STRING_TYPES_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/rosidl_adapter_proto__visibility_control.h"
#include "builtin_interfaces/rosidl_adapter_proto__visibility_control.h"
#include "test_msgs/msg/Strings.pb.h"
#include "test_msgs/msg/WStrings.pb.h"


rclcpp::SubscriptionBase::SharedPtr subscribe_strings(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::Strings>> & expected_messages,
  std::vector<bool> & received_messages);

rclcpp::SubscriptionBase::SharedPtr subscribe_wstrings(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<std::shared_ptr<test_msgs::msg::pb::WStrings>> & expected_messages,
  std::vector<bool> & received_messages);

#endif  // SUBSCRIBE_STRING_TYPES_HPP_
