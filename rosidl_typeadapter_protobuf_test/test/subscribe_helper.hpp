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

#ifndef SUBSCRIBE_HELPER_HPP_
#define SUBSCRIBE_HELPER_HPP_

#include <google/protobuf/util/message_differencer.h>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

template<typename T>
rclcpp::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<typename std::shared_ptr<typename rclcpp::TypeAdapter<T>::custom_type>>
  & expected_messages,
  std::vector<bool> & received_messages)
{
  received_messages.assign(expected_messages.size(), false);

  auto callback =
    [&expected_messages, &received_messages](
    const std::shared_ptr<typename rclcpp::TypeAdapter<T>::custom_type> received_message
    ) -> void
    {
      google::protobuf::util::MessageDifferencer mdiff;
      mdiff.set_message_field_comparison(
        google::protobuf::util::MessageDifferencer::MessageFieldComparison::EQUAL);
      mdiff.set_scope(google::protobuf::util::MessageDifferencer::Scope::PARTIAL);

      // find received message in vector of expected messages
      auto received = received_messages.begin();
      bool known_message = false;
      size_t index = 0;
      for (auto expected_message : expected_messages) {
        if (!expected_message->ByteSize()) {
          if (!received_message->ByteSize() ) {
            *received = true;
            printf("received message #%zu of %zu\n", index + 1, expected_messages.size());
            known_message = true;
            break;
          }
        } else if (mdiff.Compare(*expected_message, *received_message)) {
          *received = true;
          printf("received message #%zu of %zu\n", index + 1, expected_messages.size());
          known_message = true;
          break;
        }
        ++received;
        ++index;
      }
      if (!known_message) {
        throw std::runtime_error("received message does not match any expected message");
      }

      // shutdown node when all expected messages have been received
      for (auto received_msg : received_messages) {
        if (!received_msg) {
          return;
        }
      }
      rclcpp::shutdown();
    };

  auto qos = rclcpp::QoS(rclcpp::KeepLast(expected_messages.size()));

  return
    node->create_subscription<T>(std::string("test/message/") + message_type, qos, callback);
}

#endif  // SUBSCRIBE_HELPER_HPP_
