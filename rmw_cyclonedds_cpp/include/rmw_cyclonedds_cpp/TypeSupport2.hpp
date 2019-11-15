//
// Created by Dan Rose on 2019-11-05.
//
#include <rosidl_generator_c/string_functions.h>
#include <rosidl_generator_c/u16string_functions.h>
#include <rosidl_typesupport_introspection_c/identifier.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <rosidl_typesupport_introspection_c/service_introspection.h>

#include <cassert>
#include <regex>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <string>

#ifndef ROS2_MASTER_ROSIDL_TYPEINFO_HPP
#define ROS2_MASTER_ROSIDL_TYPEINFO_HPP
namespace rmw_cyclonedds_cpp
{
using byte = unsigned char;

namespace tsi_enum
{
using namespace rosidl_typesupport_introspection_cpp;
}
// these are shared between c and cpp
enum class ValueType : uint8_t {
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

enum class MemberContainerType { Array, Sequence, SingleValue };

template <typename UnaryFunction>
constexpr auto apply_to_primitive_value(UnaryFunction fn, ValueType value_type, void * data);

struct RTI_C
{
  static auto get_typesupport_identifier()
  {
    return rosidl_typesupport_introspection_c__identifier;
  }
  static constexpr auto & identifier = rosidl_typesupport_introspection_c__identifier;

  using MetaMessage = rosidl_typesupport_introspection_c__MessageMembers;
  using MetaMember = rosidl_typesupport_introspection_c__MessageMember;
  using MetaService = rosidl_typesupport_introspection_c__ServiceMembers;

  // wrappers to make these more stringlike
  struct String : protected rosidl_generator_c__String
  {
    using traits_type = std::char_traits<char>;

    auto size() const { return rosidl_generator_c__String::size; }
    auto begin() const { return data; }
    auto end() const { return data + size(); }
  };
  static_assert(
    sizeof(String) == sizeof(rosidl_generator_c__String), "String should not add any new members");

  struct WString : rosidl_generator_c__U16String
  {
    using traits_type = std::char_traits<char16_t>;

    auto size() const { return rosidl_generator_c__U16String::size; }

    struct iterator
    {
      const WString & str;
      size_t position;
      iterator & operator++()
      {
        position++;
        return *this;
      };

      char16_t operator*()
      {
        if (str.size() <= position) {
          throw std::out_of_range("string index out of range");
        }
        return static_cast<char16_t>(str.data[position]);
      }

      bool operator==(const iterator & other)
      {
        assert(&str == &other.str);
        return position == other.position;
      }
      bool operator!=(const iterator & other) { return !(*this == other); }
    };

    auto begin() const { return iterator{*this, 0}; }
    auto end() const { return iterator{*this, size()}; }
  };

  static_assert(
    sizeof(WString) == sizeof(rosidl_generator_c__U16String),
    "WString should not add any new members");

  template <typename T>
  struct Sequence
  {
    T * data_;
    size_t size_;
    size_t capacity_;

    T &operator [](size_t index){
      assert (index<size());
      return data_[index];
    };

    size_t size() const { return size_; }
    auto begin() { return data_; }
    auto end() { return data_ + size_; }
  };
};

struct RTI_Cpp
{
  static constexpr auto & identifier = rosidl_typesupport_introspection_cpp::typesupport_identifier;

  static auto get_typesupport_identifier()
  {
    return rosidl_typesupport_introspection_cpp::typesupport_identifier;
  }
  using MetaMessage = rosidl_typesupport_introspection_cpp::MessageMembers;
  using MetaMember = rosidl_typesupport_introspection_cpp::MessageMember;
  using MetaService = rosidl_typesupport_introspection_cpp::ServiceMembers;
};

template <typename UnaryFunction>
auto with_typesupport(const rosidl_message_type_support_t * untyped_typesupport, UnaryFunction f)
{
  const rosidl_message_type_support_t * ts;

  if ((ts = get_message_typesupport_handle(untyped_typesupport, RTI_C::identifier))) {
    return f(*static_cast<const RTI_C::MetaMessage *>(ts->data));
  } else if ((ts = get_message_typesupport_handle(untyped_typesupport, RTI_Cpp::identifier))) {
    return f(*static_cast<const RTI_Cpp::MetaMessage *>(ts->data));
  }
  throw std::runtime_error("typesupport not recognized");
}

template <typename UnaryFunction>
auto with_typesupport(const rosidl_service_type_support_t * untyped_typesupport, UnaryFunction f)
{
  const rosidl_service_type_support_t * ts;

  if ((ts = get_service_typesupport_handle(untyped_typesupport, RTI_C::identifier))) {
    return f(*static_cast<const RTI_C::MetaService *>(ts->data));
  } else if ((ts = get_service_typesupport_handle(untyped_typesupport, RTI_Cpp::identifier))) {
    return f(*static_cast<const RTI_Cpp::MetaService *>(ts->data));
  }
  throw std::runtime_error("typesupport not recognized");
}

std::pair<rosidl_message_type_support_t, rosidl_message_type_support_t>
get_svc_request_response_typesupports(const rosidl_service_type_support_t * svc);

//////////////////
template <typename MetaMessage>
struct MessageRef;

template <typename MetaMember>
struct MemberRef;

template <typename MessageMembers>
struct MessageType
{
  MessageMembers mm;

  size_t sizeof_value() const { return mm.size_of_; }
  using value_type = void;
  using reference_type = MessageRef<MessageMembers>;

  template <typename UnaryFunction>
  auto with_value(void * ptr_to_value, UnaryFunction f)
  {
    return f(make_member_ref(mm, ptr_to_value));
  };
};

template <typename UnaryFunction>
auto with_typed_ptr(
  const void * value_data, const RTI_C::MetaMember & meta_member, UnaryFunction f);

template <typename UnaryFunction>
auto with_typed_ptr(
  const void * value_data, const RTI_Cpp::MetaMember & meta_member, UnaryFunction f);

template <typename MetaMessage>
struct MessageRef
{
  using MetaMember = std::remove_pointer_t<decltype(MetaMessage::members_)>;

  const MetaMessage & meta_message;
  const void * data;

  MessageRef(const MetaMessage & meta_message, const void * data)
  : meta_message(meta_message), data(data){};
  MessageRef() = delete;

  size_t size() const { return meta_message.member_count_; }

  auto at(size_t index) const;
};

template <typename T>
struct ArrayRef
{
  ArrayRef() = delete;

  T * data;
  size_t n;

  T &operator [](size_t index){
    assert (index<size());
    return data[index];
  };

  auto size() const { return n; }
};

template <typename MetaMessage>
struct ArrayOfMessageRef
{
  ArrayOfMessageRef() = delete;

  void * data;
  size_t n;
  const MetaMessage & meta_message;

  MessageRef<MetaMessage> operator[](size_t index);

  auto size() const { return n; }
};


template <typename MetaMember>
struct MemberRef
{
  MemberRef(const MetaMember & meta_member, void * data) : meta_member(meta_member), data(data)
  {
    assert(data);
  }

  MemberRef() = delete;
  const MetaMember & meta_member;
  void * data;

  MemberContainerType get_container_type() const
  {
    if (!meta_member.is_array_) return MemberContainerType::SingleValue;
    if (meta_member.is_array_ && !meta_member.is_upper_bound_) return MemberContainerType::Array;
    if (meta_member.is_array_ && (!meta_member.array_size_ || meta_member.is_upper_bound_))
      return MemberContainerType::Sequence;
    throw std::runtime_error("Collection type not recognized");
  };

  template <typename UnaryFunction, typename Result = void>
  Result with_sequence(UnaryFunction f);

  template <typename UnaryFunction, typename Result = void>
  Result with_array(UnaryFunction f);

  template <typename UnaryFunction, typename Result = void>
  Result with_single_value(UnaryFunction f);

  bool is_submessage_type() { return ValueType(meta_member.type_id_) == ValueType ::MESSAGE; }

  template <typename UnaryFunction>
  auto with_submessage_typesupport(UnaryFunction f)
  {
    assert(is_submessage_type());
    assert(meta_member.members_);
    with_typesupport(meta_member.members_, f);
  }
};

template <typename MetaMessage>
auto make_message_ref(const MetaMessage & meta, const void * data)
{
  return MessageRef<MetaMessage>{meta, data};
}
template <typename MetaMember>
auto make_member_ref(const MetaMember & meta, void * data)
{
  return std::move(MemberRef<MetaMember>{meta, data});
}

template <typename MetaMember>
auto make_member_ref(const MetaMember & meta, const void * data)
{
  return std::move(MemberRef<MetaMember>{meta, data});
}

template <typename UnaryFunction>
auto with_message(
  const rosidl_message_type_support_t * type_support, const void * data, UnaryFunction f)
{
  return with_typesupport(type_support, [&](auto meta) { return f(make_message_ref(meta, data)); });
}

template <typename MetaService>
auto make_service_request_ref(const MetaService & meta, const void * data)
{
  return make_message_ref(meta.request_members_, data);
}

template <typename MetaService>
auto make_service_response_ref(const MetaService & meta, const void * data)
{
  return make_message_ref(meta.response_members_, data);
}

template <typename MetaMessage>
auto MessageRef<MetaMessage>::at(size_t index) const
{
  if (index >= meta_message.member_count_) {
    throw std::out_of_range("index out of range");
  }
  auto & member = meta_message.members_[index];
  return make_member_ref(member, (byte *)data + member.offset_);
}

}  // namespace rmw_cyclonedds_cpp
#include "TypeSupport2_impl.hpp"
#endif  // ROS2_MASTER_ROSIDL_TYPEINFO_HPP
