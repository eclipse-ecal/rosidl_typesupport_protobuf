# ========================= RMW eCAL LICENSE =================================
#
# Copyright (C) 2019 - 2020 Continental Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ========================= RMW eCAL LICENSE =================================

set(supported_distros
	dashing
	eloquent
	foxy
	galactic
	humble
  rolling
)

list(FIND supported_distros $ENV{ROS_DISTRO} index)
if(index EQUAL -1)
  message(FATAL_ERROR "'$ENV{ROS_DISTRO}' is an unsupported ros2 distro.")
endif()

list(FIND supported_distros foxy foxy_index)

if(${index} GREATER_EQUAL ${foxy_index})
  set(is_foxy_or_greater true)
else()
  set(is_foxy_or_greater false)
endif()


set(distro_index 0)
foreach(distro ${supported_distros})
  add_compile_definitions("$<UPPER_CASE:${distro}>=${distro_index}")
  MATH(EXPR distro_index "${distro_index}+1")
endforeach()

add_compile_definitions("ROS_DISTRO=$<UPPER_CASE:$ENV{ROS_DISTRO}>")
