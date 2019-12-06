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

#include "CDRCursor.hpp"
namespace rmw_cyclonedds_cpp
{

struct StructValueTypeMembersIterator{
  void * obj;
  const StructValueType * vt;
  Member * which_member;
};

struct SequenceIterator{
  void * obj;
  const StructValueType * vt;
  Member * which_member;
};

struct ArrayIterator{
  void * obj;
  const ArrayValueType * vt;
  Member * which_member;
};

void deserialize_top_level(void * destination_object, const void * data, const StructValueType * ts);

void deserialize(void * destination_object, const AnyValueType * vt, CDRCursor * data_cursor){
}
void deserialize(void * destination_object, const PrimitiveValueType & vt, CDRCursor * data_cursor){
}
void deserialize(void * destination_object, const U8StringValueType & vt, CDRCursor * data_cursor){
}
void deserialize(void * destination_object, const U16StringValueType & vt, CDRCursor * data_cursor){
}
void deserialize(void * destination_object, const ArrayValueType & vt, CDRCursor * data_cursor){
}
void deserialize(void * destination_object, const SpanSequenceValueType & vt, CDRCursor * data_cursor){
}
void deserialize(void * destination_object, const StructValueType & vt, CDRCursor * data_cursor){
  for (size_t i=0; i<vt.n_members(); i++){
    auto member = vt.get_member(i);
    auto member_object = byte_offset( destination_object , member->member_offset);
    deserialize(member_object, member->value_type, data_cursor);
  }
}
}