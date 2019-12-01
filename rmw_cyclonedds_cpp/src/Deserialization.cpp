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
#include "Deserialization.hpp"

#include <array>

#include "TypeSupport2.hpp"
#include "bytewise.hpp"
#include "cdr.hpp"

namespace rmw_cyclonedds_cpp
{
class CDRReader
{
public:
  endian stream_endian;
  const EncodingVersion eversion;
  const size_t max_align;

};

void deserialize(
  void * data_object, const rosidl_message_type_support_t * type_support,
  const void * serialized_message)
{

  auto ts = from_rosidl(type_support);

  std::array<byte, 4> rtps_header = {};
  for (size_t i = 0; i < 4; i++) {
    rtps_header[i] = ((const byte *)(serialized_message))[i];
  }
}

}  // namespace rmw_cyclonedds_cpp
