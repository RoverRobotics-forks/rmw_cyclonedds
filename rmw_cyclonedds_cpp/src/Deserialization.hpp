// Copyright 2019 Rover Robotics via Dan Rose
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
#ifndef DESERIALIZATION_HPP_
#define DESERIALIZATION_HPP_

#include <unordered_map>
#include <utility>

#include "rmw_cyclonedds_cpp/serdata.hpp"
#include "rosidl_generator_c/service_type_support_struct.h"

namespace rmw_cyclonedds_cpp
{
void deserialize(
  void * data_object, const rosidl_message_type_support_t * type_support,
  const void * serialized_message);
}
#endif  // DESERIALIZATION_HPP_
