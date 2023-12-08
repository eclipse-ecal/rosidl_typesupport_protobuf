/* ========================= RMW eCAL LICENSE =================================
*
* Copyright (C) 2019 - 2020 Continental Corporation
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* ========================= RMW eCAL LICENSE =================================
*/

// From Foxy distro rosidl_generator_c has been renamed to rosidl_generator_c,
// purpose of this header file is to alias old C type names to new ones, that way the
// rest of the code doesn't have to care about old package name regardless of distro.
// any rosidl_generator_c to rosidl_runtime_c adaptions should be added here

#pragma once

#include <rosidl_runtime_c/primitives_sequence.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/u16string.h>
#include <rosidl_runtime_c/u16string_functions.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
