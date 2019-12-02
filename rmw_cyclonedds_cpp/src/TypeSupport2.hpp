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
#pragma once

#include <rcl/allocator.h>

#include <cassert>
#include <regex>
#include <string>
#include <type_traits>

#include "bytewise.hpp"
#include "rosidl_generator_c/string_functions.h"
#include "rosidl_generator_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

namespace rmw_cyclonedds_cpp
{
template<class T>
struct type_identity
{
  using type = T;
};

template<typename T>
constexpr std::allocator<T> get_allocator(type_identity<T> && = std::declval<type_identity<T>>())
{
  return {};
}

enum class TypeGenerator
{
  ROSIDL_C,
  ROSIDL_Cpp,
};

TypeGenerator identify_typesupport(const char * identifier);

template<TypeGenerator>
struct TypeGeneratorInfo;

struct AnyValueType;

/// contiguous storage objects
template<typename T>
class TypedSpan;
class UntypedSpan;

template<typename T>
class TypedSpan
{
  const T * m_data;
  const size_t m_size;

public:
  TypedSpan(const T * data, size_t size)
  : m_data(data), m_size(size) {}

  size_t size() const {return m_size;}
  size_t size_bytes() const {return size() * sizeof(T);}
  const T * data() const {return m_data;}

  auto begin() {return m_data;}
  auto end() {return m_data + size();}
};

class UntypedSpan
{
protected:
  const void * m_data;
  size_t m_size_bytes;

public:
  UntypedSpan(const void * data, size_t size_bytes)
  : m_data(data), m_size_bytes(size_bytes) {}

  template<typename T>
  TypedSpan<T> cast() const
  {
    assert(m_size_bytes % sizeof(T) == 0);
    return {static_cast<T *>(m_data), m_size_bytes / sizeof(T)};
  }
  size_t size_bytes() const {return m_size_bytes;}
  const void * data() const {return m_data;}
};

class ChunkedIterator
{
protected:
  void * m_data;
  size_t m_size_bytes;

public:
  ChunkedIterator & operator++()
  {
    m_data = byte_offset(m_data, m_size_bytes);
    return *this;
  }
  UntypedSpan operator*() const {return {m_data, m_size_bytes};}
  bool operator==(const ChunkedIterator & other) const
  {
    assert(m_size_bytes == other.m_size_bytes);
    return m_data == other.m_data;
  }
  bool operator!=(const ChunkedIterator & other) const {return !(*this == other);}
};

template<typename NativeType>
auto make_typed_span(const NativeType * m_data, size_t size)
{
  return TypedSpan<NativeType>{m_data, size};
}

template<typename T>
class Casts
{
public:
  using type = T;

  static type * do_static_cast(void * ptr) {return static_cast<type *>(ptr);}
  static const type * do_static_cast(const void * ptr) {return static_cast<const type *>(ptr);}

  static type * do_dynamic_cast(void * ptr) {return dynamic_cast<type *>(ptr);}
  static const type * do_dynamic_cast(const void * ptr) {return dynamic_cast<const type *>(ptr);}
};

template<typename T>
Casts<T> casts(type_identity<T>)
{
  return {};
}

enum class EValueType
{
  // the logical value type
  PrimitiveValueType,
  U8StringValueType,
  U16StringValueType,
  StructValueType,
  ArrayValueType,
  SpanSequenceValueType,
  BoolVectorValueType,
};

struct AnyValueType
{
  // represents not just the IDL value but also its physical representation
  virtual ~AnyValueType() = default;

  // LOGICAL PROPERTIES
  // represents the logical value type and supports the 'apply' function
  virtual EValueType e_value_type() const = 0;

  // faster alternative to dynamic cast
  template<typename UnaryFunction>
  auto apply(UnaryFunction f) const;

  template<typename UnaryFunction>
  auto apply(UnaryFunction f);

  template<typename UnaryFunction>
  auto apply_to_concrete_type(UnaryFunction f) const
  {
    throw std::logic_error("this should be implemented in a subclass");
  }
};

struct InstantiableValueType : public AnyValueType
{
  virtual size_t sizeof_type() const;
  virtual size_t alignof_type() const;
  virtual void ctor(void * ptr) const;
  virtual void dtor(void * ptr) const;
};

template<typename T>
struct ConcreteValueType : public InstantiableValueType
{
  using concrete_type = T;

  // how many bytes this value type takes up
  size_t sizeof_type() const final {return sizeof(T);}
  size_t alignof_type() const final {return alignof(T);}
  void ctor(void * ptr) const final
  {
    auto x = new (ptr) T();
    after_ctor(x);
  }
  void dtor(void * ptr) const final
  {
    auto p = static_cast<concrete_type *>(ptr);
    before_dtor(p);
    p->~T();
  }

  T * cast_ptr(void * p) const {return static_cast<T *>(p);}
  const T * cast_ptr(const void * p) const {return static_cast<const T *>(p);}

  virtual void after_ctor(concrete_type *) const {}
  virtual void before_dtor(concrete_type *) const {}
};

template<>
struct TypeGeneratorInfo<TypeGenerator::ROSIDL_C>
{
  static constexpr auto enum_value = TypeGenerator::ROSIDL_C;
  static constexpr auto & identifier = rosidl_typesupport_introspection_c__identifier;
  using MetaMessage = rosidl_typesupport_introspection_c__MessageMembers;
  using MetaMember = rosidl_typesupport_introspection_c__MessageMember;
  using MetaService = rosidl_typesupport_introspection_c__ServiceMembers;
};

template<>
struct TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>
{
  static constexpr auto enum_value = TypeGenerator::ROSIDL_Cpp;
  static constexpr auto & identifier = rosidl_typesupport_introspection_cpp::typesupport_identifier;
  using MetaMessage = rosidl_typesupport_introspection_cpp::MessageMembers;
  using MetaMember = rosidl_typesupport_introspection_cpp::MessageMember;
  using MetaService = rosidl_typesupport_introspection_cpp::ServiceMembers;
};

constexpr const char * get_identifier(TypeGenerator g)
{
  switch (g) {
    case TypeGenerator::ROSIDL_C:
      return TypeGeneratorInfo<TypeGenerator::ROSIDL_C>::identifier;
    case TypeGenerator::ROSIDL_Cpp:
      return TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>::identifier;
  }
}

template<typename UnaryFunction>
void with_typesupport_info(const char * identifier, UnaryFunction f)
{
  {
    using tgi = TypeGeneratorInfo<TypeGenerator::ROSIDL_C>;
    if (identifier == tgi::identifier) {
      return f(tgi{});
    }
  }
  {
    using tgi = TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>;
    if (identifier == tgi::identifier) {
      return f(tgi{});
    }
  }
  {
    using tgi = TypeGeneratorInfo<TypeGenerator::ROSIDL_C>;
    if (std::strcmp(identifier, tgi::identifier) == 0) {
      return f(tgi{});
    }
  }
  {
    using tgi = TypeGeneratorInfo<TypeGenerator::ROSIDL_Cpp>;
    if (std::strcmp(identifier, tgi::identifier) == 0) {
      return f(tgi{});
    }
  }
  throw std::runtime_error("typesupport not recognized");
}

template<TypeGenerator g>
using MetaMessage = typename TypeGeneratorInfo<g>::MetaMessage;
template<TypeGenerator g>
using MetaMember = typename TypeGeneratorInfo<g>::MetaMember;
template<TypeGenerator g>
using MetaService = typename TypeGeneratorInfo<g>::MetaService;

namespace tsi_enum = rosidl_typesupport_introspection_cpp;

// these are shared between c and cpp
enum class ROSIDL_TypeKind : uint8_t
{
  FLOAT = tsi_enum::ROS_TYPE_FLOAT,
  DOUBLE = tsi_enum::ROS_TYPE_DOUBLE,
  LONG_DOUBLE = tsi_enum::ROS_TYPE_LONG_DOUBLE,
  CHAR = tsi_enum::ROS_TYPE_CHAR,
  WCHAR = tsi_enum::ROS_TYPE_WCHAR,
  BOOLEAN = tsi_enum::ROS_TYPE_BOOLEAN,
  OCTET = tsi_enum::ROS_TYPE_OCTET,
  UINT8 = tsi_enum::ROS_TYPE_UINT8,
  INT8 = tsi_enum::ROS_TYPE_INT8,
  UINT16 = tsi_enum::ROS_TYPE_UINT16,
  INT16 = tsi_enum::ROS_TYPE_INT16,
  UINT32 = tsi_enum::ROS_TYPE_UINT32,
  INT32 = tsi_enum::ROS_TYPE_INT32,
  UINT64 = tsi_enum::ROS_TYPE_UINT64,
  INT64 = tsi_enum::ROS_TYPE_INT64,
  STRING = tsi_enum::ROS_TYPE_STRING,
  WSTRING = tsi_enum::ROS_TYPE_WSTRING,

  MESSAGE = tsi_enum::ROS_TYPE_MESSAGE,
};

class StructValueType;
const StructValueType * from_rosidl(const rosidl_message_type_support_t * mts);

struct Member
{
  const char * name;
  const AnyValueType * value_type;
  size_t member_offset;
  size_t next_member_offset;

  const void * get_member_data(const void * ptr_to_struct)
  {
    return byte_offset(ptr_to_struct, member_offset);
  }
};

class StructValueType : public InstantiableValueType
{
public:
  ROSIDL_TypeKind type_kind() const {return ROSIDL_TypeKind::MESSAGE;}
  size_t sizeof_type() const final {return sizeof_struct();}
  virtual size_t sizeof_struct() const = 0;
  virtual size_t n_members() const = 0;
  virtual const Member * get_member(size_t) const = 0;
  EValueType e_value_type() const final {return EValueType::StructValueType;}
  size_t alignof_type() const final {return alignof(max_align_t);}
};

class ArrayValueType : public InstantiableValueType
{
protected:
  const InstantiableValueType * m_element_value_type;
  size_t m_size;

public:
  ArrayValueType(const InstantiableValueType * element_value_type, size_t size)
  : m_element_value_type(element_value_type), m_size(size)
  {
  }
  const AnyValueType * element_value_type() const {return m_element_value_type;}
  size_t sizeof_type() const final {return m_size * m_element_value_type->sizeof_type();}
  size_t array_size() const {return m_size;}
  const void * get_data(const void * ptr_to_array) const {return ptr_to_array;}
  EValueType e_value_type() const final {return EValueType::ArrayValueType;}
  void ctor(void * ptr) const override
  {
    for (size_t i = 0; i < array_size(); i++) {
      m_element_value_type->ctor(byte_offset(ptr, m_element_value_type->sizeof_type() * i));
    }
  }
  void dtor(void * ptr) const override
  {
    for (size_t i = 0; i < array_size(); i++) {
      m_element_value_type->dtor(byte_offset(ptr, m_element_value_type->sizeof_type() * i));
    }
  }
};

class SpanSequenceValueType
{
public:
  virtual const InstantiableValueType * element_value_type() const = 0;
  virtual size_t sequence_size(const void * ptr_to_sequence) const = 0;
  virtual const void * sequence_contents(const void * ptr_to_sequence) const = 0;
  virtual EValueType e_value_type() const final {return EValueType::SpanSequenceValueType;}
  virtual void resize(void *, size_t) const = 0;
};

class CallbackSpanSequenceValueType : public SpanSequenceValueType
{
protected:
  const InstantiableValueType * m_element_value_type;
  std::function<size_t(const void *)> m_size_function;
  std::function<const void * (const void *, size_t index)> m_get_const_function;

public:
  CallbackSpanSequenceValueType(
    const InstantiableValueType * element_value_type, decltype(m_size_function) size_function,
    decltype(m_get_const_function) get_const_function)
  : m_element_value_type(element_value_type),
    m_size_function(size_function),
    m_get_const_function(get_const_function)
  {
    assert(m_element_value_type);
    assert(size_function);
    assert(get_const_function);
  }
  const InstantiableValueType * element_value_type() const override {return m_element_value_type;}
  size_t sequence_size(const void * ptr_to_sequence) const override
  {
    return m_size_function(ptr_to_sequence);
  }
  const void * sequence_contents(const void * ptr_to_sequence) const override
  {
    return m_get_const_function(ptr_to_sequence, 0);
  }
};

struct ROSIDLC_SequenceObject
{
  void * data;
  size_t size;     /*!< The number of valid items in data */
  size_t capacity; /*!< The number of allocated items in data */
};

class ROSIDLC_CallbackSpanSequenceValueType : public CallbackSpanSequenceValueType
{
protected:
  decltype(MetaMember<TypeGenerator::ROSIDL_C>::resize_function) m_resize_function;

public:
  ROSIDLC_CallbackSpanSequenceValueType(
    const InstantiableValueType * element_value_type, const MetaMember<TypeGenerator::ROSIDL_C> & m)
  : CallbackSpanSequenceValueType(element_value_type, m.size_function, m.get_const_function),
    m_resize_function(m.resize_function)
  {
  }

  void resize(void * obj, size_t count) const override
  {
    if (!m_resize_function(obj, count)) {
      throw std::runtime_error("could not resize");
    }
  }
};

class ROSIDLCPP_CallbackSpanSequenceValueType : public CallbackSpanSequenceValueType
{
protected:
  decltype(MetaMember<TypeGenerator::ROSIDL_Cpp>::resize_function) m_resize_function;

public:
  ROSIDLCPP_CallbackSpanSequenceValueType(
    const InstantiableValueType * element_value_type,
    const MetaMember<TypeGenerator::ROSIDL_Cpp> & m)
  : CallbackSpanSequenceValueType(element_value_type, m.size_function, m.get_const_function),
    m_resize_function(m.resize_function)
  {
  }

  void resize(void * obj, size_t count) const override {m_resize_function(obj, count);}
};

class ROSIDLC_SpanSequenceValueType : public ConcreteValueType<ROSIDLC_SequenceObject>,
  public SpanSequenceValueType
{
protected:
  const InstantiableValueType * m_element_value_type;

public:
  explicit ROSIDLC_SpanSequenceValueType(const InstantiableValueType * element_value_type)
  : m_element_value_type(element_value_type)
  {
  }

  const InstantiableValueType * element_value_type() const override {return m_element_value_type;}
  size_t sequence_size(const void * ptr_to_sequence) const override
  {
    return cast_ptr(ptr_to_sequence)->size;
  }

  const void * sequence_contents(const void * ptr_to_sequence) const final
  {
    return cast_ptr(ptr_to_sequence)->data;
  }

  void resize(void * ptr_to_sequence, size_t count) const final
  {
    auto & seq = *cast_ptr(ptr_to_sequence);

    auto new_data = std::malloc(m_element_value_type->sizeof_type() * count);
    auto new_size = count;
    for (size_t i = 0; i < new_size; i++) {
      m_element_value_type->ctor(byte_offset(new_data, i * m_element_value_type->sizeof_type()));
    }

    auto old_data = seq.data;
    auto old_size = seq.size;
    for (size_t i = 0; i < old_size; i++) {
      m_element_value_type->dtor(
        byte_offset(old_data, i * m_element_value_type->sizeof_type()));
    }
    std::free(old_data);

    seq.size = seq.capacity = new_size;
    seq.data = new_data;
  }
};

struct PrimitiveValueType : public InstantiableValueType
{
  const ROSIDL_TypeKind m_type_kind;

  explicit constexpr PrimitiveValueType(ROSIDL_TypeKind type_kind)
  : m_type_kind(type_kind)
  {
    assert(type_kind != ROSIDL_TypeKind::STRING);
    assert(type_kind != ROSIDL_TypeKind::WSTRING);
    assert(type_kind != ROSIDL_TypeKind::MESSAGE);
  }

  ROSIDL_TypeKind type_kind() const {return m_type_kind;}

  template<typename UnaryFunction>
  auto apply_to_concrete_type(UnaryFunction f) const
  {
    switch (type_kind()) {
      case ROSIDL_TypeKind::FLOAT:
        return f(type_identity<float>{});
      case ROSIDL_TypeKind::DOUBLE:
        return f(type_identity<double>{});
      case ROSIDL_TypeKind::LONG_DOUBLE:
        return f(type_identity<long double>{});
      case ROSIDL_TypeKind::CHAR:
        return f(type_identity<char>{});
      case ROSIDL_TypeKind::WCHAR:
        return f(type_identity<char16_t>{});
      case ROSIDL_TypeKind::BOOLEAN:
        return f(type_identity<bool>{});
      case ROSIDL_TypeKind::OCTET:  // NOLINT(bugprone-branch-clone)
        return f(type_identity<unsigned char>{});
      case ROSIDL_TypeKind::UINT8:
        return f(type_identity<uint_least8_t>{});
      case ROSIDL_TypeKind::INT8:
        return f(type_identity<int8_t>{});
      case ROSIDL_TypeKind::UINT16:
        return f(type_identity<uint16_t>{});
      case ROSIDL_TypeKind::INT16:
        return f(type_identity<int16_t>{});
      case ROSIDL_TypeKind::UINT32:
        return f(type_identity<uint32_t>{});
      case ROSIDL_TypeKind::INT32:
        return f(type_identity<int32_t>{});
      case ROSIDL_TypeKind::UINT64:
        return f(type_identity<uint64_t>{});
      case ROSIDL_TypeKind::INT64:
        return f(type_identity<int64_t>{});
      case ROSIDL_TypeKind::STRING:
      case ROSIDL_TypeKind::WSTRING:
      case ROSIDL_TypeKind::MESSAGE:
        throw std::logic_error("not a primitive");
    }
  }

  void ctor(void * ptr) const override
  {
    return apply_to_concrete_type([&](auto t) {
               using T = typename decltype(t)::type;
               new (ptr) T();
             });
  }

  void dtor(void * ptr) const override
  {
    return apply_to_concrete_type([&](auto t) {
               using T = typename decltype(t)::type;
               static_cast<T *>(ptr)->~T();
             });
  }

  size_t sizeof_type() const final
  {
    return apply_to_concrete_type([&](auto t) {return sizeof(typename decltype(t)::type);});
  }

  size_t alignof_type() const final
  {
    return apply_to_concrete_type([&](auto t) {return alignof(typename decltype(t)::type);});
  }

  EValueType e_value_type() const override {return EValueType::PrimitiveValueType;}
};

class BoolVectorValueType : public ConcreteValueType<std::vector<bool>>
{
protected:
  static std::unique_ptr<PrimitiveValueType> s_element_value_type;

public:
  static const AnyValueType * element_value_type()
  {
    if (!s_element_value_type) {
      s_element_value_type = std::make_unique<PrimitiveValueType>(ROSIDL_TypeKind::BOOLEAN);
    }
    return s_element_value_type.get();
  }

  std::vector<bool>::const_iterator begin(const void * ptr_to_sequence) const
  {
    return cast_ptr(ptr_to_sequence)->begin();
  }
  std::vector<bool>::const_iterator end(const void * ptr_to_sequence) const
  {
    return cast_ptr(ptr_to_sequence)->end();
  }
  size_t size(const void * ptr_to_sequence) const {return cast_ptr(ptr_to_sequence)->size();}
  EValueType e_value_type() const final {return EValueType::BoolVectorValueType;}
  virtual void resize(void * obj, size_t count) const final {return cast_ptr(obj)->resize(count);}
};

class U8StringValueType
{
public:
  using char_traits = std::char_traits<char>;
  ROSIDL_TypeKind type_kind() const {return ROSIDL_TypeKind::STRING;}
  virtual TypedSpan<char_traits::char_type> data(void *) const = 0;
  virtual TypedSpan<const char_traits::char_type> data(const void *) const = 0;
  virtual EValueType e_value_type() const final {return EValueType::U8StringValueType;}
  virtual void assign(void * obj, const char_traits::char_type * s, size_t count) const = 0;
};

class U16StringValueType
{
public:
  using char_traits = std::char_traits<char16_t>;
  ROSIDL_TypeKind type_kind() const {return ROSIDL_TypeKind::WSTRING;}
  virtual TypedSpan<char_traits::char_type> data(void *) const = 0;
  virtual TypedSpan<const char_traits::char_type> data(const void *) const = 0;
  virtual EValueType e_value_type() const final {return EValueType::U16StringValueType;}
  virtual void assign(void * obj, const char_traits::char_type * s, size_t count) const = 0;
};

struct ROSIDLC_StringValueType
  : public ConcreteValueType<rosidl_generator_c__String>, public U8StringValueType
{
public:
  TypedSpan<const char_traits::char_type> data(const void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {str->data, str->size};
  }
  TypedSpan<char_traits::char_type> data(void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {str->data, str->size};
  }

  void after_ctor(concrete_type * ptr) const override
  {
    if (!rosidl_generator_c__String__init(ptr)) {
      throw std::runtime_error("failed to construct string");
    }
  }
  void before_dtor(concrete_type * ptr) const override
  {
    rosidl_generator_c__String__fini(ptr);
  }

  void assign(void * obj, const char_traits::char_type * s, size_t count) const override
  {
    rosidl_generator_c__String__assignn(cast_ptr(obj), s, count);
  }
};

class ROSIDLC_WStringValueType : public ConcreteValueType<rosidl_generator_c__U16String>,
  public U16StringValueType
{
public:
  TypedSpan<const char_traits::char_type> data(const void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {reinterpret_cast<const char_traits::char_type *>(str->data), str->size};
  }
  TypedSpan<char_traits::char_type> data(void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {reinterpret_cast<char_traits::char_type *>(str->data), str->size};
  }

  void after_ctor(concrete_type * ptr) const override
  {
    if (!rosidl_generator_c__U16String__init(ptr)) {
      throw std::runtime_error("failed to construct string");
    }
  }
  void before_dtor(concrete_type * ptr) const override
  {
    rosidl_generator_c__U16String__fini(ptr);
  }

  void assign(void * obj, const char_traits::char_type * s, size_t count) const override
  {
    rosidl_generator_c__U16String__assignn(
      cast_ptr(obj), reinterpret_cast<const uint16_t *>(s), count);
  }
};

class ROSIDLCPP_StringValueType : public ConcreteValueType<std::string>, public U8StringValueType
{
public:
  TypedSpan<const char_traits::char_type> data(const void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {str->data(), str->size()};
  }
  TypedSpan<char_traits::char_type> data(void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {str->data(), str->size()};
  }
  void assign(void * ptr, const char_traits::char_type * s, size_t count) const override
  {
    cast_ptr(ptr)->assign(s, count);
  }
};

class ROSIDLCPP_U16StringValueType : public ConcreteValueType<std::u16string>,
  public U16StringValueType
{
public:
  TypedSpan<const char_traits::char_type> data(const void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {str->data(), str->size()};
  }
  TypedSpan<char_traits::char_type> data(void * ptr) const override
  {
    auto str = cast_ptr(ptr);
    return {str->data(), str->size()};
  }

  void assign(void * ptr, const char_traits::char_type * s, size_t count) const override
  {
    cast_ptr(ptr)->assign(s, count);
  }
};

template<typename UnaryFunction>
auto AnyValueType::apply(UnaryFunction f) const
{
  switch (e_value_type()) {
    case EValueType::PrimitiveValueType:
      return f(*dynamic_cast<const PrimitiveValueType *>(this));
    case EValueType::U8StringValueType:
      return f(*dynamic_cast<const U8StringValueType *>(this));
    case EValueType::U16StringValueType:
      return f(*dynamic_cast<const U16StringValueType *>(this));
    case EValueType::StructValueType:
      return f(*dynamic_cast<const StructValueType *>(this));
    case EValueType::ArrayValueType:
      return f(*dynamic_cast<const ArrayValueType *>(this));
    case EValueType::SpanSequenceValueType:
      return f(*dynamic_cast<const SpanSequenceValueType *>(this));
    case EValueType::BoolVectorValueType:
      return f(*dynamic_cast<const BoolVectorValueType *>(this));
  }
}

template<typename UnaryFunction>
auto AnyValueType::apply(UnaryFunction f)
{
  switch (e_value_type()) {
    case EValueType::PrimitiveValueType:
      return f(*dynamic_cast<PrimitiveValueType *>(this));
    case EValueType::U8StringValueType:
      return f(*dynamic_cast<U8StringValueType *>(this));
    case EValueType::U16StringValueType:
      return f(*dynamic_cast<U16StringValueType *>(this));
    case EValueType::StructValueType:
      return f(*dynamic_cast<StructValueType *>(this));
    case EValueType::ArrayValueType:
      return f(*dynamic_cast<ArrayValueType *>(this));
    case EValueType::SpanSequenceValueType:
      return f(*dynamic_cast<SpanSequenceValueType *>(this));
    case EValueType::BoolVectorValueType:
      return f(*dynamic_cast<BoolVectorValueType *>(this));
  }
}

}  // namespace rmw_cyclonedds_cpp
